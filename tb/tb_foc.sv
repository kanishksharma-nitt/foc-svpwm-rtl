// Self-checking testbench for the FOC pipeline.
//
// Replays the closed-loop stimulus recorded by python/foc_model.py (the
// golden model ran the same integer controller against a float PMSM plant)
// and checks, on every control strobe, that the DUT's registered Park
// outputs (id/iq) and all three SVPWM duties match the golden values
// bit-exactly. Afterwards it checks the dead-time generator: complementary
// gates never both high, and the measured high-side on-time over one full
// PWM period equals duty - DT - 1.
`timescale 1ns / 1ps
`default_nettype none

module tb_foc;

  localparam CLK_PERIOD = 10;
  localparam DT = 8;

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg signed [15:0] ia = 16'sd0, ib = 16'sd0;
  reg [15:0] theta = 16'd0;
  reg signed [15:0] spd_fb = 16'sd0, spd_ref = 16'sd0;

  wire ctrl_strobe;
  wire signed [15:0] id_meas, iq_meas;
  wire [9:0] duty_a, duty_b, duty_c;
  wire [2:0] sector;
  wire gh_a, gl_a, gh_b, gl_b, gh_c, gl_c;

  integer errors = 0;
  integer nsteps, k, idx, n, hi_cnt;
  reg [31:0] vec [0:4095];
  reg signed [15:0] exp_id, exp_iq;
  reg [9:0] exp_da, exp_db, exp_dc;

  foc_top #(.CTRL_DIV(50), .SPEED_DIV(20), .DT(DT)) dut (
      .clk(clk), .rst_n(rst_n),
      .ia(ia), .ib(ib), .theta(theta),
      .spd_fb(spd_fb), .spd_ref(spd_ref),
      .ctrl_strobe(ctrl_strobe),
      .id_meas(id_meas), .iq_meas(iq_meas),
      .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c),
      .sector(sector),
      .gh_a(gh_a), .gl_a(gl_a),
      .gh_b(gh_b), .gl_b(gl_b),
      .gh_c(gh_c), .gl_c(gl_c)
  );

  always #(CLK_PERIOD / 2) clk = ~clk;

  // complementary gates must never conduct simultaneously (any phase)
  always @(posedge clk) begin
    if (rst_n) begin
      if ((gh_a && gl_a) || (gh_b && gl_b) || (gh_c && gl_c)) begin
        errors = errors + 1;
        $display("FAIL: complementary gates both high at %0t", $time);
      end
    end
  end

  task apply(input integer i);
    begin
      ia      = vec[i][15:0];
      ib      = vec[i+1][15:0];
      theta   = vec[i+2][15:0];
      spd_fb  = vec[i+3][15:0];
      spd_ref = vec[i+4][15:0];
    end
  endtask

  initial begin
    if ($test$plusargs("vcd")) begin
      $dumpfile("tb_foc.vcd");
      $dumpvars(0, tb_foc);
    end

    $readmemh("../test/foc_vectors.mem", vec);
    nsteps = vec[0];

    repeat (5) @(posedge clk);
    rst_n = 1'b1;

    // ---------------- dead-time / PWM timing check -----------------------
    // With all-zero inputs every PI error is zero, so the duty is provably
    // stable at 512 (svpwm(0,0)). Measure phase A over one full 1024-cycle
    // counter period: on-time must be exactly duty - DT - 1.
    @(negedge clk);
    while (dut.pwm_cnt != 10'd0) @(negedge clk);
    hi_cnt = 0;
    for (n = 0; n < 1024; n = n + 1) begin
      if (gh_a) hi_cnt = hi_cnt + 1;
      @(negedge clk);
    end
    if (hi_cnt !== 512 - DT - 1) begin
      errors = errors + 1;
      $display("FAIL: phase-A on-time %0d, expected %0d (duty 512, DT %0d)",
               hi_cnt, 512 - DT - 1, DT);
    end else begin
      $display("dead-time check: on-time %0d = duty 512 - DT %0d - 1",
               hi_cnt, DT);
    end

    // present step-0 stimulus before the next control strobe
    apply(1);

    for (k = 0; k < nsteps; k = k + 1) begin
      idx = 1 + 10 * k;

      @(negedge clk);
      while (!ctrl_strobe) @(negedge clk);
      @(negedge clk);   // step-k results are now registered

      exp_id = vec[idx+5][15:0];
      exp_iq = vec[idx+6][15:0];
      exp_da = vec[idx+7][9:0];
      exp_db = vec[idx+8][9:0];
      exp_dc = vec[idx+9][9:0];

      if (id_meas !== exp_id || iq_meas !== exp_iq) begin
        errors = errors + 1;
        if (errors < 20)
          $display("FAIL step %0d: id/iq got (%0d,%0d) expected (%0d,%0d)",
                   k, id_meas, iq_meas, exp_id, exp_iq);
      end
      if (duty_a !== exp_da || duty_b !== exp_db || duty_c !== exp_dc) begin
        errors = errors + 1;
        if (errors < 20)
          $display("FAIL step %0d: duty got (%0d,%0d,%0d) expected (%0d,%0d,%0d)",
                   k, duty_a, duty_b, duty_c, exp_da, exp_db, exp_dc);
      end

      if (k + 1 < nsteps) apply(1 + 10 * (k + 1));
    end
    $display("control-loop compare done: %0d steps, %0d errors",
             nsteps, errors);

    if (errors == 0) $display("TEST PASSED");
    else             $display("TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #3_000_000;
    $display("TEST FAILED: global timeout");
    $finish;
  end

endmodule

`default_nettype wire
