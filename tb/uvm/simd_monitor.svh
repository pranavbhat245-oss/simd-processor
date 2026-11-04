// ---------------------------------------------------------------------------
// Monitor (passive)
//   * in_ap  : every accepted input  (valid_in && ready_in)
//   * out_ap : every consumed output (valid_out && ready_out)
//   * rst_ap : 1 on the first clock edge seen in reset (scoreboard flushes)
//   Keeps the cycle / stall counters so the scoreboard can check latency.
//   Pop is processed before push on the same edge.
// ---------------------------------------------------------------------------
class simd_monitor extends uvm_monitor;
  `uvm_component_utils(simd_monitor)

  virtual simd_if vif;
  uvm_analysis_port #(simd_txn)     in_ap;
  uvm_analysis_port #(simd_out_txn) out_ap;
  uvm_analysis_port #(bit)          rst_ap;

  longint unsigned cyc   = 0;
  longint unsigned stall = 0;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_ap  = new("in_ap",  this);
    out_ap = new("out_ap", this);
    rst_ap = new("rst_ap", this);
    if (!uvm_config_db#(virtual simd_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("MON", "virtual interface 'vif' not found in config_db")
    end
  endfunction

  task run_phase(uvm_phase phase);
    bit in_reset = 1'b0;
    forever begin
      @(vif.mon_cb);
      cyc++;
      if (vif.rst === 1'b1) begin
        if (!in_reset) begin
          rst_ap.write(1'b1);
          in_reset = 1'b1;
        end
      end
      else begin
        in_reset = 1'b0;
        if (vif.mon_cb.valid_out === 1'b1 && vif.mon_cb.ready_out === 1'b1) sample_out();
        if (vif.mon_cb.valid_in  === 1'b1 && vif.mon_cb.ready_in  === 1'b1) sample_in();
        if (vif.mon_cb.valid_out === 1'b1 && vif.mon_cb.ready_out === 1'b0) stall++;
      end
    end
  endtask

  function void sample_in();
    simd_txn t;
    t = simd_txn::type_id::create("mon_in");
    t.opcode      = vif.mon_cb.opcode;
    t.signed_mode = vif.mon_cb.signed_mode;
    t.relu_en     = vif.mon_cb.relu_en;
    t.mask        = vif.mon_cb.mask;
    t.c           = vif.mon_cb.c;
    for (int i = 0; i < 4; i++) begin
      t.a[i] = vif.mon_cb.a[i];
      t.b[i] = vif.mon_cb.b[i];
    end
    t.acc_cycle = cyc;
    t.acc_stall = stall;
    in_ap.write(t);
  endfunction

  function void sample_out();
    simd_out_txn o;
    o = simd_out_txn::type_id::create("mon_out");
    for (int i = 0; i < 4; i++) begin
      o.lane_res[i] = vif.mon_cb.lane_result[i];
      o.code[i]     = vif.mon_cb.exception_code[i];
    end
    o.lane_valid  = vif.mon_cb.lane_valid_out;
    o.dot         = vif.mon_cb.dot_result;
    o.mac         = vif.mon_cb.mac_result;
    o.sad         = vif.mon_cb.sad_result;
    o.red_valid   = vif.mon_cb.reduction_valid_out;
    o.opcode      = vif.mon_cb.opcode_out;
    o.signed_mode = vif.mon_cb.signed_mode_out;
    o.dz          = vif.mon_cb.div_by_zero;
    o.ov          = vif.mon_cb.overflow;
    o.un          = vif.mon_cb.underflow;
    o.sinv        = vif.mon_cb.sqrt_invalid;
    o.exc         = vif.mon_cb.exception_out;
    o.done_cycle  = cyc;
    o.done_stall  = stall;
    out_ap.write(o);
  endfunction
endclass
