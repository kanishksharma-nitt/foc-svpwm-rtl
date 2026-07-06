// FOC pipeline top: Clarke -> Park -> d/q PI -> inverse Park -> SVPWM ->
// dead-time, with a multi-rate structure on one clock:
//   current loop : every CTRL_DIV clock cycles (ctrl strobe)
//   speed loop   : every SPEED_DIV ctrl strobes
//
// The transform/PI/SVPWM datapath is combinational and registered once on the
// ctrl strobe. The speed PI output (iq_ref) is registered, so a ctrl strobe
// that also updates the speed loop uses the previous iq_ref; the golden model
// mirrors this ordering.
`default_nettype none

module foc_top #(
    parameter CTRL_DIV  = 50,
    parameter SPEED_DIV = 20,
    parameter DT        = 8,
    parameter LUT_FILE  = "../rtl/sin_lut.mem"
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] ia,        // phase currents, Q1.15
    input  wire signed [15:0] ib,
    input  wire [15:0]        theta,     // rotor angle, 0..2^16-1 = 0..2*pi
    input  wire signed [15:0] spd_fb,    // speed feedback, Q1.15
    input  wire signed [15:0] spd_ref,
    output wire               ctrl_strobe,
    output reg  signed [15:0] id_meas,   // registered Park outputs (debug)
    output reg  signed [15:0] iq_meas,
    output reg  [9:0]         duty_a,
    output reg  [9:0]         duty_b,
    output reg  [9:0]         duty_c,
    output wire [2:0]         sector,
    output wire               gh_a, gl_a,
    output wire               gh_b, gl_b,
    output wire               gh_c, gl_c
);

  // ------------------------------------------------------------- strobes
  reg [7:0] divcnt;
  reg [4:0] scnt;
  assign ctrl_strobe = (divcnt == CTRL_DIV - 1);
  wire speed_strobe  = ctrl_strobe && (scnt == SPEED_DIV - 1);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      divcnt <= 8'd0;
      scnt   <= 5'd0;
    end else begin
      divcnt <= ctrl_strobe ? 8'd0 : divcnt + 8'd1;
      if (ctrl_strobe)
        scnt <= (scnt == SPEED_DIV - 1) ? 5'd0 : scnt + 5'd1;
    end
  end

  // ------------------------------------------------------ transform chain
  wire signed [15:0] sin_t, cos_t;
  sincos #(.LUT_FILE(LUT_FILE)) u_sincos (
      .theta(theta), .sin_o(sin_t), .cos_o(cos_t));

  wire signed [15:0] i_alpha, i_beta, id_c, iq_c;
  clarke u_clarke (.ia(ia), .ib(ib), .i_alpha(i_alpha), .i_beta(i_beta));
  park   u_park   (.i_alpha(i_alpha), .i_beta(i_beta),
                   .sin_t(sin_t), .cos_t(cos_t), .id(id_c), .iq(iq_c));

  // --------------------------------------------------------------- loops
  reg signed [15:0] iq_ref;
  wire signed [15:0] vd, vq, iq_ref_next;

  pi_ctrl #(.KP(16'sd4096), .KI(16'sd205)) u_pi_d (
      .clk(clk), .rst_n(rst_n), .en(ctrl_strobe),
      .ref_i(16'sd0), .fb(id_c), .u(vd));
  pi_ctrl #(.KP(16'sd4096), .KI(16'sd205)) u_pi_q (
      .clk(clk), .rst_n(rst_n), .en(ctrl_strobe),
      .ref_i(iq_ref), .fb(iq_c), .u(vq));
  pi_ctrl #(.KP(16'sd8192), .KI(16'sd205)) u_pi_w (
      .clk(clk), .rst_n(rst_n), .en(speed_strobe),
      .ref_i(spd_ref), .fb(spd_fb), .u(iq_ref_next));

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            iq_ref <= 16'sd0;
    else if (speed_strobe) iq_ref <= iq_ref_next;
  end

  // ------------------------------------------------------- output stage
  wire signed [15:0] v_alpha, v_beta;
  inv_park u_inv_park (.vd(vd), .vq(vq), .sin_t(sin_t), .cos_t(cos_t),
                       .v_alpha(v_alpha), .v_beta(v_beta));

  wire [9:0] da_c, db_c, dc_c;
  svpwm u_svpwm (.v_alpha(v_alpha), .v_beta(v_beta),
                 .da(da_c), .db(db_c), .dc(dc_c), .sector(sector));

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      id_meas <= 16'sd0;
      iq_meas <= 16'sd0;
      duty_a  <= 10'd512;
      duty_b  <= 10'd512;
      duty_c  <= 10'd512;
    end else if (ctrl_strobe) begin
      id_meas <= id_c;
      iq_meas <= iq_c;
      duty_a  <= da_c;
      duty_b  <= db_c;
      duty_c  <= dc_c;
    end
  end

  // ------------------------------------------------- PWM + dead time
  reg [9:0] pwm_cnt;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) pwm_cnt <= 10'd0;
    else        pwm_cnt <= pwm_cnt + 10'd1;
  end

  wire raw_a = (pwm_cnt < duty_a);
  wire raw_b = (pwm_cnt < duty_b);
  wire raw_c = (pwm_cnt < duty_c);

  deadtime #(.DT(DT)) u_dt_a (.clk(clk), .rst_n(rst_n), .raw(raw_a),
                              .g_hi(gh_a), .g_lo(gl_a));
  deadtime #(.DT(DT)) u_dt_b (.clk(clk), .rst_n(rst_n), .raw(raw_b),
                              .g_hi(gh_b), .g_lo(gl_b));
  deadtime #(.DT(DT)) u_dt_c (.clk(clk), .rst_n(rst_n), .raw(raw_c),
                              .g_hi(gh_c), .g_lo(gl_c));

endmodule

`default_nettype wire
