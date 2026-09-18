`timescale 1ns/1ns

// Self-checking testbench. Each case runs the solver on one graph. After every
// Euler step it compares the hardware phases with a floating-point model of that
// step, computed from the hardware's own phases before the step.
module testbench;
    reg clock = 1'b0;
    reg reset = 1'b1;
    always #5 clock = ~clock;

    localparam NUM_CASES = 6;
    wire [NUM_CASES-1:0] finished;
    wire [NUM_CASES-1:0] passed;

    // Same graph as the default in kuramoto_solver.v
    localparam [99:0] ORIGINAL_ADJ = {
        10'b0010100100, 10'b0000001010, 10'b1000000000, 10'b0000000000, 10'b1000000000,
        10'b0000000000, 10'b0100000010, 10'b1000000000, 10'b0100001000, 10'b0000000000
    };

    // Cycle 0-1-...-(n-1)-0 plus the chords i -- i+n/2
    function [99:0] mobius_ladder(input integer n);
        integer i;
        begin
            mobius_ladder = 100'd0;
            for (i = 0; i < n; i = i + 1) begin
                mobius_ladder[i*n + (i+1)%n]     = 1'b1;
                mobius_ladder[((i+1)%n)*n + i]   = 1'b1;
                mobius_ladder[i*n + (i+n/2)%n]   = 1'b1;
                mobius_ladder[((i+n/2)%n)*n + i] = 1'b1;
            end
        end
    endfunction

    function [99:0] complete_graph(input integer n);
        integer i, k;
        begin
            complete_graph = 100'd0;
            for (i = 0; i < n; i = i + 1)
                for (k = 0; k < n; k = k + 1)
                    if (i != k) complete_graph[i*n + k] = 1'b1;
        end
    endfunction

    kuramoto_case #(.NAME("original-10"), .N(10), .ADJ(ORIGINAL_ADJ))
        case0 (.clock(clock), .reset(reset), .finished(finished[0]), .passed(passed[0]));
    kuramoto_case #(.NAME("mobius-4"), .N(4), .ADJ(mobius_ladder(4)))
        case1 (.clock(clock), .reset(reset), .finished(finished[1]), .passed(passed[1]));
    kuramoto_case #(.NAME("mobius-6"), .N(6), .ADJ(mobius_ladder(6)))
        case2 (.clock(clock), .reset(reset), .finished(finished[2]), .passed(passed[2]));
    kuramoto_case #(.NAME("mobius-8"), .N(8), .ADJ(mobius_ladder(8)))
        case3 (.clock(clock), .reset(reset), .finished(finished[3]), .passed(passed[3]));
    kuramoto_case #(.NAME("mobius-10"), .N(10), .ADJ(mobius_ladder(10)))
        case4 (.clock(clock), .reset(reset), .finished(finished[4]), .passed(passed[4]));
    kuramoto_case #(.NAME("complete-10"), .N(10), .ADJ(complete_graph(10)))
        case5 (.clock(clock), .reset(reset), .finished(finished[5]), .passed(passed[5]));

    initial begin
        #20 reset = 1'b0;
        wait (&finished);
        #1;
        if (&passed) $display("RESULT: PASS (all %0d cases)", NUM_CASES);
        else         $display("RESULT: FAIL (passed mask %b)", passed);
        $stop;
    end
endmodule


module kuramoto_case #(
    parameter NAME = "case",
    parameter N = 10,
    parameter ITERATIONS = 1000,
    parameter [N*N-1:0] ADJ = {N*N{1'b0}},
    parameter CLOCK_PERIOD = 10
)(
    input  wire clock,
    input  wire reset,
    output reg  finished,
    output reg  passed
);
    localparam IDX_W = $clog2(N);
    localparam real SCALE    = 536870912.0;   // 2^29
    localparam real PI_R     = 3.14159265358979323846;
    localparam real TWO_PI_R = 6.28318530717958647692;
    localparam real K_EDGE   = 0.5;
    localparam real K_SHIL   = 0.5;
    localparam real DT       = 0.00390625;    // 2^-8
    localparam real STEP_TOL = 1.0e-7;
    localparam real INIT_TOL = 1.0e-8;
    localparam integer TIMEOUT = ITERATIONS * N * 64 + 1000;

    reg              start = 1'b0;
    reg [IDX_W-1:0]  phase_sel = {IDX_W{1'b0}};
    wire             done;
    wire [N-1:0]     partition;
    wire [31:0]      phase_out;

    kuramoto_solver #(.N(N), .ITERATIONS(ITERATIONS), .ADJ(ADJ)) dut (
        .clock(clock), .reset(reset), .start(start), .phase_sel(phase_sel),
        .done(done), .partition(partition), .phase_out(phase_out)
    );

    function real hw_phase(input integer i);
        hw_phase = dut.phases_flat[i*32 +: 32] / SCALE;
    endfunction

    function real wrap(input real x);
        begin
            wrap = x;
            while (wrap < 0.0)       wrap = wrap + TWO_PI_R;
            while (wrap >= TWO_PI_R) wrap = wrap - TWO_PI_R;
        end
    endfunction

    function real circ_dist(input real a, input real b);
        real d;
        begin
            d = wrap(a - b);
            circ_dist = (d > PI_R) ? TWO_PI_R - d : d;
        end
    endfunction

    function integer cut_of(input [N-1:0] p);
        integer a, b;
        begin
            cut_of = 0;
            for (a = 0; a < N; a = a + 1)
                for (b = a + 1; b < N; b = b + 1)
                    if (ADJ[a*N + b] && (p[a] != p[b])) cut_of = cut_of + 1;
        end
    endfunction

    // One-step check against the floating-point model
    real    pre [0:N-1];
    real    f, expected, err, max_step_err;
    integer ci, cj, steps_checked, errors, cycles_per_step;
    time    last_update_time;

    always @(posedge clock) begin
        if (dut.update) begin
            if (steps_checked > 0)
                cycles_per_step = ($time - last_update_time) / CLOCK_PERIOD;
            last_update_time = $time;
            for (ci = 0; ci < N; ci = ci + 1) pre[ci] = hw_phase(ci);
            @(negedge clock);
            for (ci = 0; ci < N; ci = ci + 1) begin
                f = 2.0 * K_SHIL * $sin(2.0 * pre[ci]);
                for (cj = 0; cj < N; cj = cj + 1)
                    if (ADJ[ci*N + cj]) f = f + K_EDGE * $sin(pre[cj] - pre[ci]);
                expected = wrap(pre[ci] - DT * f);
                err = circ_dist(expected, hw_phase(ci));
                if (err > max_step_err) max_step_err = err;
                if (err > STEP_TOL || dut.phases_flat[ci*32 +: 32] >= 32'd3373259426) begin
                    errors = errors + 1;
                    if (errors <= 5)
                        $display("[%0s] step %0d node %0d: hardware %.9f rad, expected %.9f rad",
                                 NAME, steps_checked + 1, ci, hw_phase(ci), expected);
                end
            end
            steps_checked = steps_checked + 1;
        end
    end

    integer     i, mask, cycles, edges, cut, best_cut;
    real        rel, settle, max_settle;
    reg [N-1:0] expected_partition;

    initial begin
        finished        = 1'b0;
        passed          = 1'b0;
        errors          = 0;
        steps_checked   = 0;
        cycles_per_step = 0;
        max_step_err    = 0.0;

        @(negedge reset);
        @(negedge clock) start = 1'b1;
        @(negedge clock) start = 1'b0;

        // Initial phases must be 2*pi*i/N + pi/5
        wait (dut.load === 1'b1);
        @(posedge clock);
        @(negedge clock);
        for (i = 0; i < N; i = i + 1)
            if (circ_dist(hw_phase(i), wrap(TWO_PI_R * i / N + PI_R / 5.0)) > INIT_TOL) begin
                errors = errors + 1;
                $display("[%0s] node %0d starts at %.9f rad, expected %.9f rad",
                         NAME, i, hw_phase(i), wrap(TWO_PI_R * i / N + PI_R / 5.0));
            end

        cycles = 0;
        while (done !== 1'b1 && cycles < TIMEOUT) begin
            @(posedge clock);
            cycles = cycles + 1;
        end

        if (done !== 1'b1) begin
            errors = errors + 1;
            $display("[%0s] no done after %0d cycles", NAME, cycles);
        end else begin
            @(negedge clock);

            if (steps_checked != ITERATIONS) begin
                errors = errors + 1;
                $display("[%0s] %0d steps checked, expected %0d", NAME, steps_checked, ITERATIONS);
            end

            // The partition output must match the final phases.
            max_settle = 0.0;
            for (i = 0; i < N; i = i + 1) begin
                rel = wrap(hw_phase(i) - hw_phase(0));
                expected_partition[i] = (rel >= PI_R / 2.0) && (rel < 1.5 * PI_R);
                settle = circ_dist(rel, expected_partition[i] ? PI_R : 0.0);
                if (settle > max_settle) max_settle = settle;
            end
            if (partition !== expected_partition) begin
                errors = errors + 1;
                $display("[%0s] partition %b does not match the phases (expected %b)",
                         NAME, partition, expected_partition);
            end

            // The read port must return each unit's phase.
            for (i = 0; i < N; i = i + 1) begin
                phase_sel = i;
                #1;
                if (phase_out !== dut.phases_flat[i*32 +: 32]) begin
                    errors = errors + 1;
                    $display("[%0s] phase_out for node %0d is wrong", NAME, i);
                end
            end

            edges = 0;
            for (i = 0; i < N*N; i = i + 1) if (ADJ[i]) edges = edges + 1;
            edges = edges / 2;
            best_cut = 0;
            for (mask = 0; mask < (1 << N); mask = mask + 1) begin
                cut = cut_of(mask);
                if (cut > best_cut) best_cut = cut;
            end
            cut = cut_of(partition);

            $write("[%0s] final phases (rad):", NAME);
            for (i = 0; i < N; i = i + 1) $write(" %.4f", hw_phase(i));
            $write("\n");
            $display("[%0s] N=%0d, %0d edges, %0d steps at %0d cycles/step, max step error %.1e rad, max distance from 0/pi %.1e rad",
                     NAME, N, edges, steps_checked, cycles_per_step, max_step_err, max_settle);
            $display("[%0s] partition %b (node %0d first) cuts %0d of %0d edges; best possible cut is %0d",
                     NAME, partition, N - 1, cut, edges, best_cut);
        end

        passed = (errors == 0);
        $display("[%0s] %0s", NAME, passed ? "PASS" : "FAIL");
        finished = 1'b1;
    end
endmodule
