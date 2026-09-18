`timescale 1ns / 1ps

// Rotation-mode CORDIC sine. Angles and results are Q3.29; theta_in must be in [0, 2*pi).
module cordic_sine #(
    parameter NUM_ITER = 32
)(
    input clk,
    input rst,
    input start,
    input [31:0] theta_in,
    output reg signed [31:0] sine_out,
    output reg done,
    output reg busy
);
  // 1/A_n = 0.607252935 in Q3.29, so the rotated vector needs no gain correction afterwards.
  localparam signed [31:0] K = 32'sd326016437;

  localparam [31:0] HALF_PI       = 32'd843314857;
  localparam [31:0] PI            = 32'd1686629713;
  localparam [31:0] THREE_HALF_PI = 32'd2529944570;
  localparam [31:0] TWO_PI        = 32'd3373259426;

  reg signed [31:0] x, y, z;
  reg negate;
  reg [5:0] iter;

  // atan(2^-i) in Q3.29
  reg signed [31:0] atan_table [0:31];
  initial begin
    atan_table[0]  = 32'sd421657428;
    atan_table[1]  = 32'sd248918914;
    atan_table[2]  = 32'sd131521918;
    atan_table[3]  = 32'sd66762579;
    atan_table[4]  = 32'sd33510843;
    atan_table[5]  = 32'sd16771757;
    atan_table[6]  = 32'sd8387925;
    atan_table[7]  = 32'sd4194218;
    atan_table[8]  = 32'sd2097141;
    atan_table[9]  = 32'sd1048574;
    atan_table[10] = 32'sd524287;
    atan_table[11] = 32'sd262143;
    atan_table[12] = 32'sd131071;
    atan_table[13] = 32'sd65535;
    atan_table[14] = 32'sd32767;
    atan_table[15] = 32'sd16383;
    atan_table[16] = 32'sd8191;
    atan_table[17] = 32'sd4095;
    atan_table[18] = 32'sd2047;
    atan_table[19] = 32'sd1023;
    atan_table[20] = 32'sd511;
    atan_table[21] = 32'sd255;
    atan_table[22] = 32'sd127;
    atan_table[23] = 32'sd63;
    atan_table[24] = 32'sd31;
    atan_table[25] = 32'sd15;
    atan_table[26] = 32'sd7;
    atan_table[27] = 32'sd4;
    atan_table[28] = 32'sd2;
    atan_table[29] = 32'sd1;
    atan_table[30] = 32'sd0;
    atan_table[31] = 32'sd0;
  end

  reg signed [31:0] x_shr, y_shr;

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      iter     <= 0;
      busy     <= 0;
      done     <= 0;
      sine_out <= 0;
      x        <= 0;
      y        <= 0;
      z        <= 0;
      negate   <= 0;
    end else begin
      if (start && !busy) begin
        // Fold the angle into [0, pi/2]; quadrants III and IV negate the result.
        if (theta_in < HALF_PI) begin
          z      <= theta_in;
          negate <= 0;
        end else if (theta_in < PI) begin
          z      <= PI - theta_in;
          negate <= 0;
        end else if (theta_in < THREE_HALF_PI) begin
          z      <= theta_in - PI;
          negate <= 1;
        end else begin
          z      <= TWO_PI - theta_in;
          negate <= 1;
        end
        x    <= K;
        y    <= 32'sd0;
        iter <= 0;
        busy <= 1;
        done <= 0;
      end else if (busy) begin
        x_shr = x >>> iter;
        y_shr = y >>> iter;

        if (z[31]) begin
          x <= x + y_shr;
          y <= y - x_shr;
          z <= z + atan_table[iter];
        end else begin
          x <= x - y_shr;
          y <= y + x_shr;
          z <= z - atan_table[iter];
        end

        iter <= iter + 1;

        if (iter == NUM_ITER - 1) begin
          busy <= 0;
          done <= 1;
          // Register the result of this last rotation directly.
          if (z[31])
            sine_out <= negate ? -(y - x_shr) : (y - x_shr);
          else
            sine_out <= negate ? -(y + x_shr) : (y + x_shr);
        end
      end else begin
        done <= 0;
      end
    end
  end

endmodule
