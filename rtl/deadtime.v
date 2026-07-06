// Dead-time insertion for one complementary PWM pair. On any edge of the
// raw PWM input both gates are dropped, then the target gate is asserted
// after the dead interval. Because change detection uses a registered copy
// of the input, the actual both-off gap is DT+1 clock cycles; the testbench
// asserts this exact relationship (high-side on-time = duty - DT - 1).
`default_nettype none

module deadtime #(
    parameter DT = 8
) (
    input  wire clk,
    input  wire rst_n,
    input  wire raw,     // desired high-side state
    output reg  g_hi,
    output reg  g_lo
);

  reg       raw_q;
  reg [7:0] cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      raw_q <= 1'b0;
      cnt   <= 8'd0;
      g_hi  <= 1'b0;
      g_lo  <= 1'b0;
    end else begin
      raw_q <= raw;
      if (raw != raw_q) begin
        g_hi <= 1'b0;
        g_lo <= 1'b0;
        cnt  <= DT[7:0];
      end else if (cnt != 8'd0) begin
        cnt <= cnt - 8'd1;
      end else begin
        g_hi <= raw_q;
        g_lo <= ~raw_q;
      end
    end
  end

endmodule

`default_nettype wire
