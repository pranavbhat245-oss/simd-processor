// ---------------------------------------------------------------------------
// Driver
//   * drives one transaction per accepted handshake, back-to-back when the
//     sequence supplies the next item in the same time step
//   * a separate process generates ready_out backpressure (cfg.ready_mode)
// Invariant: every drive is issued right after a clock edge (see align/tick).
// ---------------------------------------------------------------------------
class simd_driver extends uvm_driver #(simd_txn);
  `uvm_component_utils(simd_driver)

  virtual simd_if vif;
  simd_cfg        cfg;
  realtime        last_edge = -1.0;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual simd_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("DRV", "virtual interface 'vif' not found in config_db")
    end
    if (!uvm_config_db#(simd_cfg)::get(this, "", "cfg", cfg)) begin
      cfg = simd_cfg::type_id::create("cfg");
    end
  endfunction

  task tick();
    @(vif.drv_cb);
    last_edge = $realtime;
  endtask

  task align();
    if ($realtime != last_edge) tick();
  endtask

  task run_phase(uvm_phase phase);
    simd_txn t;
    wait (vif.rst === 1'b0);
    tick();
    fork
      ready_gen();
    join_none
    forever begin
      seq_item_port.get_next_item(t);
      align();
      if (t.gap > 0) begin
        vif.drv_cb.valid_in <= 1'b0;
        repeat (t.gap) tick();
      end
      if (t.mask == 4'b0000) drive_zero_mask(t);
      else                   drive_one(t);
      // default to idle; if the sequence hands over the next item in this very
      // time step, its drive overrides this one (last drive in a time step wins)
      vif.drv_cb.valid_in <= 1'b0;
      seq_item_port.item_done();
    end
  endtask

  task apply(simd_txn t);
    bit [3:0][7:0] av, bv;
    for (int i = 0; i < 4; i++) begin
      av[i] = t.a[i];
      bv[i] = t.b[i];
    end
    vif.drv_cb.valid_in    <= 1'b1;
    vif.drv_cb.opcode      <= t.opcode;
    vif.drv_cb.signed_mode <= t.signed_mode;
    vif.drv_cb.relu_en     <= t.relu_en;
    vif.drv_cb.mask        <= t.mask;
    vif.drv_cb.a           <= av;
    vif.drv_cb.b           <= bv;
    vif.drv_cb.c           <= t.c;
  endtask

  // Present t and hold it until ready_in is seen high at a clock edge.
  task drive_one(simd_txn t);
    int unsigned waited = 0;
    apply(t);
    do begin
      tick();
      waited++;
      if (waited > cfg.max_wait) begin
        `uvm_fatal("DRV", $sformatf("not accepted after %0d clocks: %s", cfg.max_wait, t.convert2string()))
      end
    end while (vif.drv_cb.ready_in !== 1'b1);
  endtask

  // mask==0 can never be accepted (ready_in depends on |mask). Check that.
  task drive_zero_mask(simd_txn t);
    apply(t);
    repeat (4) begin
      tick();
      if (vif.drv_cb.ready_in === 1'b1) begin
        `uvm_error("DRV", "ready_in asserted for mask==0 (DUT behaviour changed - update scoreboard/driver)")
      end
    end
    `uvm_info("DRV", "mask==0 transaction was (correctly) never accepted; dropped by driver", UVM_MEDIUM)
  endtask

  task ready_gen();
    bit          cur   = 1'b1;
    int unsigned burst = 0;
    forever begin
      tick_only();
      case (cfg.ready_mode)
        0:       cur = 1'b1;
        1:       cur = ($urandom_range(3, 0) != 0);
        2:       cur = ($urandom_range(3, 0) == 0);
        default: begin
          if (burst == 0) begin
            cur   = ~cur;
            burst = $urandom_range(15, 0) + 1;
          end
          else burst--;
        end
      endcase
      vif.drv_cb.ready_out <= cur;
    end
  endtask

  // separate clock wait for ready_gen (must not touch last_edge used by main loop)
  task tick_only();
    @(vif.drv_cb);
  endtask
endclass
