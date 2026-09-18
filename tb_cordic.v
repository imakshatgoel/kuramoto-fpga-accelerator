`timescale 1ns/1ps

// Checks cordic_sine against $sin at the quadrant edges and on a sweep of [0, 2*pi).
module tb_cordic;
    localparam real    SCALE = 536870912.0;   // 2^29
    localparam real    TOL   = 1.0e-7;
    localparam integer SWEEP = 4096;
    localparam [31:0]  HALF_PI       = 32'd843314857;
    localparam [31:0]  PI            = 32'd1686629713;
    localparam [31:0]  THREE_HALF_PI = 32'd2529944570;
    localparam [31:0]  TWO_PI        = 32'd3373259426;

    reg                clk   = 1'b0;
    reg                rst   = 1'b1;
    reg                start = 1'b0;
    reg  [31:0]        theta = 32'd0;
    wire signed [31:0] sine;
    wire               done;
    wire               busy;

    cordic_sine uut (
        .clk(clk), .rst(rst), .start(start), .theta_in(theta),
        .sine_out(sine), .done(done), .busy(busy)
    );

    always #5 clk = ~clk;

    integer k;
    real    err, max_err, worst_angle, at_half_pi;

    task run_angle(input [31:0] t);
        begin
            @(negedge clk);
            theta = t;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            @(posedge done);
            @(negedge clk);
            err = sine / SCALE - $sin(t / SCALE);
            if (err < 0.0) err = -err;
            if (err > max_err) begin
                max_err     = err;
                worst_angle = t / SCALE;
            end
        end
    endtask

    initial begin
        max_err     = 0.0;
        worst_angle = 0.0;
        #20 rst = 1'b0;
        run_angle(HALF_PI);
        at_half_pi = sine / SCALE;
        run_angle(32'd0);
        run_angle(PI);
        run_angle(THREE_HALF_PI);
        run_angle(HALF_PI - 1);
        run_angle(PI - 1);
        run_angle(THREE_HALF_PI - 1);
        run_angle(TWO_PI - 1);
        for (k = 0; k < SWEEP; k = k + 1)
            run_angle((TWO_PI / SWEEP) * k);
        $display("cordic_sine: sin(pi/2) = %.9f, max |error| = %.3e at %.6f rad over %0d angles",
                 at_half_pi, max_err, worst_angle, SWEEP + 8);
        if (max_err < TOL) $display("RESULT: PASS");
        else               $display("RESULT: FAIL (tolerance %.1e)", TOL);
        $stop;
    end
endmodule
