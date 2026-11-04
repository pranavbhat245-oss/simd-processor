class simd_agent extends uvm_agent;
  `uvm_component_utils(simd_agent)

  uvm_sequencer #(simd_txn) sqr;
  simd_driver               drv;
  simd_monitor              mon;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    sqr = uvm_sequencer #(simd_txn)::type_id::create("sqr", this);
    drv = simd_driver::type_id::create("drv", this);
    mon = simd_monitor::type_id::create("mon", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    drv.seq_item_port.connect(sqr.seq_item_export);
  endfunction
endclass

class simd_env extends uvm_env;
  `uvm_component_utils(simd_env)

  simd_agent      agent;
  simd_scoreboard sb;
  simd_coverage   cov;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent = simd_agent::type_id::create("agent", this);
    sb    = simd_scoreboard::type_id::create("sb", this);
    cov   = simd_coverage::type_id::create("cov", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agent.mon.in_ap.connect(sb.in_imp);
    agent.mon.out_ap.connect(sb.out_imp);
    agent.mon.rst_ap.connect(sb.rst_imp);
    agent.mon.in_ap.connect(cov.in_imp);
    agent.mon.out_ap.connect(cov.out_imp);
  endfunction
endclass
