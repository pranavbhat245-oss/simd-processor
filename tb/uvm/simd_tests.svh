// ---------------------------------------------------------------------------
// Tests.   Run with  +UVM_TESTNAME=<name>
//   plusargs:  +N_TXN=<n>  (random tests)   +READY_MODE=<0..3>
// ---------------------------------------------------------------------------
class simd_base_test extends uvm_test;
  `uvm_component_utils(simd_base_test)

  simd_env        env;
  simd_cfg        cfg;
  virtual simd_if vif;
  int unsigned    n_txn = 5000;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    cfg = simd_cfg::type_id::create("cfg");
    void'($value$plusargs("N_TXN=%d", n_txn));
    void'($value$plusargs("READY_MODE=%d", cfg.ready_mode));
    uvm_config_db#(simd_cfg)::set(this, "env*", "cfg", cfg);
    env = simd_env::type_id::create("env", this);
    if (!uvm_config_db#(virtual simd_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("TEST", "virtual interface 'vif' not found in config_db")
    end
  endfunction

  function void end_of_elaboration_phase(uvm_phase phase);
    super.end_of_elaboration_phase(phase);
    uvm_top.print_topology();
  endfunction

  // Let the last transactions travel through the 10-stage pipeline.
  task drain(int unsigned max_cycles = 2000);
    cfg.ready_mode = 0;
    repeat (15) @(posedge vif.clk);
    for (int unsigned i = 0; i < max_cycles && env.sb.pending() != 0; i++) @(posedge vif.clk);
    repeat (5) @(posedge vif.clk);
  endtask

  function void report_phase(uvm_phase phase);
    uvm_report_server svr;
    super.report_phase(phase);
    if (vif.sva_fail_cnt != 0) begin
      `uvm_error("SVA", $sformatf("%0d interface assertion failures (see $error messages above)", vif.sva_fail_cnt))
    end
    svr = uvm_report_server::get_server();
    if (svr.get_severity_count(UVM_ERROR) + svr.get_severity_count(UVM_FATAL) == 0)
      `uvm_info("RESULT", "*** TEST PASSED ***", UVM_NONE)
    else
      `uvm_info("RESULT", "*** TEST FAILED ***", UVM_NONE)
  endfunction
endclass

// Every opcode x both modes x a grid of corner values + exception corners.
class simd_smoke_test extends simd_base_test;
  `uvm_component_utils(simd_smoke_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_corner_seq s;
    s = simd_corner_seq::type_id::create("s");
    phase.raise_objection(this);
    s.start(env.agent.sqr);
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Constrained-random traffic with idle gaps.
class simd_random_test extends simd_base_test;
  `uvm_component_utils(simd_random_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_rand_seq s;
    s = simd_rand_seq::type_id::create("s");
    s.n = n_txn;
    phase.raise_objection(this);
    s.start(env.agent.sqr);
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Same random traffic under all four ready_out patterns, back-to-back (no gaps).
class simd_backpressure_test extends simd_base_test;
  `uvm_component_utils(simd_backpressure_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_rand_seq s;
    phase.raise_objection(this);
    for (int unsigned m = 0; m < 4; m++) begin
      cfg.ready_mode = m;
      s = simd_rand_seq::type_id::create($sformatf("s%0d", m));
      s.n = n_txn;
      s.no_gap = 1;
      s.start(env.agent.sqr);
    end
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Every lane mask, partial-lane exceptions, plus the mask==0 behaviour.
class simd_mask_test extends simd_base_test;
  `uvm_component_utils(simd_mask_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_mask_seq      m;
    simd_zero_mask_seq z;
    m = simd_mask_seq::type_id::create("m");
    z = simd_zero_mask_seq::type_id::create("z");
    phase.raise_objection(this);
    m.start(env.agent.sqr);
    z.start(env.agent.sqr);
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Reset in the middle of traffic; the scoreboard flushes and checking resumes.
class simd_reset_test extends simd_base_test;
  `uvm_component_utils(simd_reset_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_rand_seq s1, s2;
    s1 = simd_rand_seq::type_id::create("s1");
    s2 = simd_rand_seq::type_id::create("s2");
    s1.n = 400;  s1.no_gap = 1;
    s2.n = 400;
    phase.raise_objection(this);
    fork
      s1.start(env.agent.sqr);
      begin
        repeat (60) @(posedge vif.clk);
        vif.pulse_reset(3);
      end
    join
    drain();
    s2.start(env.agent.sqr);
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Regression: all 65536 pairs for every 2-operand opcode, both modes, plus
// every value for the 1-operand opcodes. Slow (~490k clocks) - run in batch.
class simd_exhaustive_test extends simd_base_test;
  `uvm_component_utils(simd_exhaustive_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_sweep_seq s;
    bit [3:0] two_op [11];
    bit [3:0] one_op [5];
    two_op = '{OPC_ADD, OPC_SUB, OPC_MULT, OPC_DIV, OPC_MIN, OPC_MAX,
               OPC_SATADD, OPC_GCD, OPC_SAD, OPC_DOT, OPC_MAC};
    one_op = '{OPC_RELU, OPC_LRELU, OPC_ABS, OPC_SQRT, OPC_POPCOUNT};
    phase.raise_objection(this);
    for (int sm = 0; sm < 2; sm++) begin
      foreach (two_op[i]) begin
        s = simd_sweep_seq::type_id::create("s");
        s.op = two_op[i];  s.sm = sm[0];  s.unary = 0;
        s.start(env.agent.sqr);
      end
      foreach (one_op[i]) begin
        s = simd_sweep_seq::type_id::create("s");
        s.op = one_op[i];  s.sm = sm[0];  s.unary = 1;
        s.start(env.agent.sqr);
      end
    end
    drain();
    phase.drop_objection(this);
  endtask
endclass

// Corner sequence AND mask sequence in one run, so a single coverage report
// covers every opcode, mode, exception and lane mask.
class simd_full_test extends simd_base_test;
  `uvm_component_utils(simd_full_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    simd_corner_seq s;
    simd_mask_seq   m;
    s = simd_corner_seq::type_id::create("s");
    m = simd_mask_seq::type_id::create("m");
    phase.raise_objection(this);
    s.start(env.agent.sqr);
    m.start(env.agent.sqr);
    drain();
    phase.drop_objection(this);
  endtask
endclass
