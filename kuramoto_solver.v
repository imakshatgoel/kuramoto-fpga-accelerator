`timescale 1ns/1ns

// Parallel Kuramoto oscillator solver for Max-Cut. Phases are unsigned Q3.29 in [0, 2*pi).
module kuramoto_solver #(
    parameter N = 10,
    parameter ITERATIONS = 1000,
    // ADJ[i*N + j] is 1 when nodes i and j share an edge, so row i is ADJ[i*N +: N].
    // It must be symmetric with a zero diagonal.
    parameter [N*N-1:0] ADJ = {
        10'b0010100100,   // node 9: 2, 5, 7
        10'b0000001010,   // node 8: 1, 3
        10'b1000000000,   // node 7: 9
        10'b0000000000,   // node 6
        10'b1000000000,   // node 5: 9
        10'b0000000000,   // node 4
        10'b0100000010,   // node 3: 1, 8
        10'b1000000000,   // node 2: 9
        10'b0100001000,   // node 1: 3, 8
        10'b0000000000    // node 0
    }
)(
    input  wire                 clock,
    input  wire                 reset,
    input  wire                 start,
    input  wire [$clog2(N)-1:0] phase_sel,
    output reg                  done,
    output reg  [N-1:0]         partition,
    output wire [31:0]          phase_out
);
    localparam IDX_W = $clog2(N);
    localparam ACC_W = 33 + IDX_W;

    localparam [31:0] HALF_PI       = 32'd843314857;
    localparam [31:0] THREE_HALF_PI = 32'd2529944570;
    localparam [31:0] TWO_PI        = 32'd3373259426;

    localparam [2:0] S_IDLE   = 3'd0,
                     S_LOAD   = 3'd1,
                     S_SLOT   = 3'd2,
                     S_ISSUE  = 3'd3,
                     S_WAIT   = 3'd4,
                     S_UPDATE = 3'd5,
                     S_CHECK  = 3'd6,
                     S_FINISH = 3'd7;

    reg  [2:0]       state;
    reg  [IDX_W-1:0] bus_owner;
    reg  [31:0]      step_count;
    reg              load;
    reg              slot_start;
    reg              update;

    wire [N*32-1:0]  phases_flat;
    wire [N-1:0]     unit_busy;

    // Shared bus: in each slot exactly one unit's phase is broadcast to all units.
    wire [31:0] bus_phase = phases_flat[bus_owner*32 +: 32];

    assign phase_out = phases_flat[phase_sel*32 +: 32];

    genvar gi, gj;
    generate
        // Build-time checks: instantiating a module that doesn't exist stops elaboration.
        if (N < 2) begin : check_n
            N_MUST_BE_AT_LEAST_2 error_n ();
        end
        for (gi = 0; gi < N; gi = gi + 1) begin : check_adj
            if (ADJ[gi*N + gi]) begin : diag
                ADJ_DIAGONAL_MUST_BE_ZERO error_diag ();
            end
            for (gj = gi + 1; gj < N; gj = gj + 1) begin : sym
                if (ADJ[gi*N + gj] != ADJ[gj*N + gi]) begin : bad
                    ADJ_MUST_BE_SYMMETRIC error_sym ();
                end
            end
        end

        for (gi = 0; gi < N; gi = gi + 1) begin : units
            computational_unit #(
                .UNIT_ID(gi), .N(N), .IDX_W(IDX_W), .ACC_W(ACC_W)
            ) cu (
                .clock(clock), .reset(reset),
                .load(load), .slot_start(slot_start), .update(update),
                .bus_owner(bus_owner), .bus_phase(bus_phase),
                .adj_row(ADJ[gi*N +: N]),
                .phase(phases_flat[gi*32 +: 32]),
                .busy(unit_busy[gi])
            );
        end
    endgenerate

    // Node j joins node 0's group when its phase relative to node 0 is nearer 0 than pi.
    reg [N-1:0] readout;
    reg [31:0]  rel;
    integer     j;
    always @* begin
        for (j = 0; j < N; j = j + 1) begin
            if (phases_flat[j*32 +: 32] >= phases_flat[31:0])
                rel = phases_flat[j*32 +: 32] - phases_flat[31:0];
            else
                rel = phases_flat[j*32 +: 32] + (TWO_PI - phases_flat[31:0]);
            readout[j] = (rel >= HALF_PI) && (rel < THREE_HALF_PI);
        end
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            state      <= S_IDLE;
            done       <= 1'b0;
            partition  <= {N{1'b0}};
            bus_owner  <= {IDX_W{1'b0}};
            step_count <= 32'd0;
            load       <= 1'b0;
            slot_start <= 1'b0;
            update     <= 1'b0;
        end else begin
            load       <= 1'b0;
            slot_start <= 1'b0;
            update     <= 1'b0;
            case (state)
                S_IDLE: if (start) begin
                    done       <= 1'b0;
                    load       <= 1'b1;
                    step_count <= 32'd0;
                    bus_owner  <= {IDX_W{1'b0}};
                    state      <= S_LOAD;
                end
                S_LOAD: state <= S_SLOT;
                S_SLOT: begin
                    slot_start <= 1'b1;
                    state      <= S_ISSUE;
                end
                // Units raise busy on the edge that ends S_ISSUE, so S_WAIT never sees a stale idle.
                S_ISSUE: state <= S_WAIT;
                S_WAIT: if (unit_busy == {N{1'b0}}) begin
                    if (bus_owner == N - 1) begin
                        update <= 1'b1;
                        state  <= S_UPDATE;
                    end else begin
                        bus_owner <= bus_owner + 1'b1;
                        state     <= S_SLOT;
                    end
                end
                S_UPDATE: begin
                    step_count <= step_count + 1'b1;
                    state      <= S_CHECK;
                end
                S_CHECK: if (step_count == ITERATIONS) begin
                    state <= S_FINISH;
                end else begin
                    bus_owner <= {IDX_W{1'b0}};
                    state     <= S_SLOT;
                end
                S_FINISH: begin
                    partition <= readout;
                    done      <= 1'b1;
                    state     <= S_IDLE;
                end
            endcase
        end
    end
endmodule
