`timescale 1ns/1ns

// One oscillator. It owns phase i and has a single CORDIC core: in its own bus
// slot it computes sin(2*theta_i), and in slot k it computes sin(theta_k - theta_i)
// when node k is a neighbour.
module computational_unit #(
    parameter UNIT_ID = 0,
    parameter N = 10,
    parameter IDX_W = 4,
    parameter ACC_W = 37,
    parameter [31:0] K_EDGE = 32'd268435456,   // 0.5 in Q3.29
    parameter [31:0] K_SHIL = 32'd268435456,   // 0.5 in Q3.29
    parameter DT_SHIFT = 8                      // dt = 2^-DT_SHIFT
)(
    input  wire             clock,
    input  wire             reset,
    input  wire             load,
    input  wire             slot_start,
    input  wire             update,
    input  wire [IDX_W-1:0] bus_owner,
    input  wire [31:0]      bus_phase,
    input  wire [N-1:0]     adj_row,
    output reg  [31:0]      phase,
    output wire             busy
);
    localparam [63:0]        TWO_PI_64  = 64'd3373259426;
    localparam [31:0]        TWO_PI     = 32'd3373259426;
    localparam [31:0]        PI_OVER_5  = 32'd337325943;
    localparam [32:0]        INIT_RAW   = (TWO_PI_64 * UNIT_ID) / N + PI_OVER_5;
    localparam [31:0]        INIT_PHASE = (INIT_RAW >= TWO_PI) ? INIT_RAW - TWO_PI : INIT_RAW;
    localparam signed [33:0] TWO_PI_S   = {2'b00, TWO_PI};
    localparam signed [63:0] K_EDGE_S   = {32'd0, K_EDGE};
    localparam signed [63:0] K_SHIL_S   = {32'd0, K_SHIL};

    wire        own_slot   = (bus_owner == UNIT_ID);
    wire        neighbour  = adj_row[bus_owner];
    wire [32:0] two_theta  = {phase, 1'b0};
    wire [31:0] shil_angle = (two_theta >= TWO_PI) ? two_theta - TWO_PI : two_theta[31:0];
    wire [31:0] diff_angle = (bus_phase >= phase) ? bus_phase - phase
                                                  : bus_phase + (TWO_PI - phase);

    reg                     job;
    reg                     job_is_shil;
    reg                     cordic_start;
    reg  [31:0]             cordic_theta;
    wire signed [31:0]      sin_val;
    wire                    cordic_done;
    reg  signed [ACC_W-1:0] coupling_sum;
    reg  signed [31:0]      shil_sin;

    cordic_sine cordic (
        .clk(clock), .rst(reset), .start(cordic_start), .theta_in(cordic_theta),
        .sine_out(sin_val), .done(cordic_done), .busy()
    );

    assign busy = job;

    // Products are formed at 64 bits so nothing is lost before the shift back to Q3.29.
    wire signed [63:0]      sin_64     = sin_val;
    wire signed [63:0]      shil_64    = shil_sin;
    wire signed [63:0]      edge_prod  = sin_64 * K_EDGE_S;
    wire signed [63:0]      shil_prod  = shil_64 * K_SHIL_S;
    wire signed [ACC_W-1:0] edge_term  = edge_prod >>> 29;
    // dtheta/dt = sum_j K_ij sin(theta_j - theta_i) + 2 K_SHIL sin(2 theta_i)
    wire signed [ACC_W-1:0] derivative = coupling_sum + (shil_prod >>> 28);
    wire signed [ACC_W-1:0] euler_step = derivative >>> DT_SHIFT;
    // Stepping against dtheta/dt pushes neighbours apart and makes 0 and pi the stable phases.
    wire signed [ACC_W:0]   next_raw   = $signed({{(ACC_W-31){1'b0}}, phase}) - euler_step;
    wire [31:0]             next_phase = (next_raw < 0)         ? next_raw + TWO_PI_S :
                                         (next_raw >= TWO_PI_S) ? next_raw - TWO_PI_S :
                                                                  next_raw[31:0];

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            phase        <= 32'd0;
            job          <= 1'b0;
            job_is_shil  <= 1'b0;
            cordic_start <= 1'b0;
            cordic_theta <= 32'd0;
            coupling_sum <= {ACC_W{1'b0}};
            shil_sin     <= 32'sd0;
        end else begin
            cordic_start <= 1'b0;
            if (load) begin
                phase        <= INIT_PHASE;
                coupling_sum <= {ACC_W{1'b0}};
            end else if (update) begin
                phase        <= next_phase;
                coupling_sum <= {ACC_W{1'b0}};
            end else if (slot_start && (own_slot || neighbour)) begin
                cordic_theta <= own_slot ? shil_angle : diff_angle;
                job_is_shil  <= own_slot;
                job          <= 1'b1;
                cordic_start <= 1'b1;
            end else if (job && cordic_done) begin
                job <= 1'b0;
                if (job_is_shil)
                    shil_sin <= sin_val;
                else
                    coupling_sum <= coupling_sum + edge_term;
            end
        end
    end
endmodule
