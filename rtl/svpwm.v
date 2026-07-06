// SVPWM duty computation via min/max common-mode injection (equivalent to
// conventional sector-based SVPWM):
//   va = v_alpha
//   vb = (-v_alpha + sqrt(3)*v_beta) / 2
//   vc = (-v_alpha - sqrt(3)*v_beta) / 2
//   vcm = (max + min)/2;  duty_x = ((vx - vcm) + 1.0) / 2  scaled to 10 bits
// Duties are clamped to [DMIN, DMAX]. The sector output (1..6) is decoded
// from the phase ordering used by the min/max search.
`default_nettype none

module svpwm #(
    parameter [9:0] DMIN = 10'd10,
    parameter [9:0] DMAX = 10'd1013
) (
    input  wire signed [15:0] v_alpha,
    input  wire signed [15:0] v_beta,
    output wire [9:0]         da,
    output wire [9:0]         db,
    output wire [9:0]         dc,
    output wire [2:0]         sector
);

  wire signed [19:0] va = {{4{v_alpha[15]}}, v_alpha};
  wire signed [32:0] t_full = v_beta * 17'sd56756;   // sqrt(3) in Q1.15
  wire signed [19:0] t  = {{2{t_full[32]}}, t_full[32:15]};   // >>> 15
  wire signed [19:0] vb = (-va + t) >>> 1;
  wire signed [19:0] vc = (-va - t) >>> 1;

  wire signed [19:0] vmax = (va >= vb) ? ((va >= vc) ? va : vc)
                                       : ((vb >= vc) ? vb : vc);
  wire signed [19:0] vmin = (va <= vb) ? ((va <= vc) ? va : vc)
                                       : ((vb <= vc) ? vb : vc);
  wire signed [19:0] vcm  = (vmax + vmin) >>> 1;

  function [9:0] duty(input signed [19:0] v, input signed [19:0] cm);
    reg signed [20:0] pre;
    begin
      pre = ((v - cm) + 21'sd32768) >>> 6;
      duty = (pre < $signed({11'd0, DMIN})) ? DMIN :
             (pre > $signed({11'd0, DMAX})) ? DMAX : pre[9:0];
    end
  endfunction

  assign da = duty(va, vcm);
  assign db = duty(vb, vcm);
  assign dc = duty(vc, vcm);

  // sector 1..6 from the ordering of (va, vb, vc)
  wire a_b = (va >= vb);
  wire b_c = (vb >= vc);
  wire a_c = (va >= vc);
  assign sector = (a_b  &&  b_c) ? 3'd1 :
                  (!a_b &&  a_c) ? 3'd2 :
                  (b_c  && !a_c) ? 3'd3 :
                  (!b_c && !a_b) ? 3'd4 :
                  (a_b  && !a_c) ? 3'd5 : 3'd6;

endmodule

`default_nettype wire
