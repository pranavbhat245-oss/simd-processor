// ---------------------------------------------------------------------------
// Scoreboard
//   Predicts every accepted transaction with simd_ref_pkg (in order) and
//   compares against what the DUT delivers. Checks, per transaction:
//     latency == 10 (clocks frozen by ready_out=0 are not counted),
//     opcode / signed flag, lane_valid == mask, lane data + exception code +
//     all five exception flags for active lanes, masked lanes quiet,
//     DOT / MAC / SAD results and reduction_valid_out.
// ---------------------------------------------------------------------------
typedef struct {
  simd_txn   t;
  bit [15:0] lane [4];
  bit [2:0]  exc  [4];
  bit [18:0] dot, mac, sad;
} simd_exp_t;

class simd_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(simd_scoreboard)

  localparam int unsigned LATENCY   = 10;
  localparam int unsigned MAX_PRINT = 40;

  uvm_analysis_imp_in  #(simd_txn,     simd_scoreboard) in_imp;
  uvm_analysis_imp_out #(simd_out_txn, simd_scoreboard) out_imp;
  uvm_analysis_imp_rst #(bit,          simd_scoreboard) rst_imp;

  simd_exp_t q[$];

  int unsigned n_in, n_out, n_flushed;
  int unsigned e_lat, e_ctrl, e_mask, e_data, e_exc, e_red, e_misc;
  int unsigned n_printed;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_imp  = new("in_imp",  this);
    out_imp = new("out_imp", this);
    rst_imp = new("rst_imp", this);
  endfunction

  function int unsigned pending();
    return q.size();
  endfunction

  function int unsigned total_errors();
    return e_lat + e_ctrl + e_mask + e_data + e_exc + e_red + e_misc;
  endfunction

  // Count every mismatch, but only print the first MAX_PRINT as uvm_error.
  function void flag(string id, string msg);
    if (n_printed < MAX_PRINT) begin
      `uvm_error(id, msg)
    end
    n_printed++;
  endfunction

  // ---------------------------------------------------------------- predict
  function void write_in(simd_txn t);
    simd_exp_t e;
    e.t = t;
    for (int i = 0; i < 4; i++) begin
      e.lane[i] = ref_lane(t.opcode, t.signed_mode, t.a[i], t.b[i]);
      e.exc[i]  = ref_exc (t.opcode, t.signed_mode, t.a[i], t.b[i]);
    end
    e.dot = ref_dot(t.signed_mode, t.relu_en, t.mask, t.a, t.b);
    e.mac = ref_mac(t.signed_mode, t.relu_en, t.mask, t.a, t.b, t.c);
    e.sad = ref_sad(t.signed_mode, t.mask, t.a, t.b);
    q.push_back(e);
    n_in++;
  endfunction

  function void write_rst(bit v);
    if (q.size() != 0) begin
      `uvm_info("SB", $sformatf("reset: flushing %0d in-flight transactions", q.size()), UVM_MEDIUM)
    end
    n_flushed += q.size();
    q.delete();
  endfunction

  // ---------------------------------------------------------------- compare
  function void write_out(simd_out_txn o);
    simd_exp_t e;
    simd_txn   t;
    longint    lat;
    string     ctx;

    if (q.size() == 0) begin
      e_misc++;
      flag("SB_EXTRA", $sformatf("DUT produced an output with nothing pending: %s", o.convert2string()));
      return;
    end
    e = q.pop_front();
    t = e.t;
    n_out++;
    ctx = t.convert2string();

    lat = longint'(o.done_cycle - t.acc_cycle) - 1 - longint'(o.done_stall - t.acc_stall);
    if (lat != LATENCY) begin
      e_lat++;
      flag("SB_LATENCY", $sformatf("latency %0d != %0d : %s", lat, LATENCY, ctx));
    end
    if (o.opcode !== t.opcode || o.signed_mode !== t.signed_mode) begin
      e_ctrl++;
      flag("SB_CTRL", $sformatf("opcode/mode exp=%s/%0b got=%s/%0b : %s",
                                ref_op_name(t.opcode), t.signed_mode, ref_op_name(o.opcode), o.signed_mode, ctx));
    end
    if (o.lane_valid !== t.mask) begin
      e_mask++;
      flag("SB_MASK", $sformatf("lane_valid exp=%04b got=%04b : %s", t.mask, o.lane_valid, ctx));
    end

    for (int i = 0; i < 4; i++) begin
      if (t.mask[i]) begin
        if (o.lane_res[i] !== e.lane[i]) begin
          e_data++;
          flag("SB_DATA", $sformatf("lane%0d exp=%04h got=%04h : %s", i, e.lane[i], o.lane_res[i], ctx));
        end
        if (o.code[i] !== e.exc[i]) begin
          e_exc++;
          flag("SB_EXC", $sformatf("lane%0d code exp=%s got=%0d : %s", i, ref_exc_name(e.exc[i]), o.code[i], ctx));
        end
        if (o.dz[i]   !== (e.exc[i] == EXC_DIVZERO)  ||
            o.ov[i]   !== (e.exc[i] == EXC_OVERFLOW) ||
            o.un[i]   !== (e.exc[i] == EXC_UNDER)    ||
            o.sinv[i] !== (e.exc[i] == EXC_INVALID)  ||
            o.exc[i]  !== (e.exc[i] != EXC_NONE)) begin
          e_exc++;
          flag("SB_FLAGS", $sformatf("lane%0d flags dz/ov/un/inv/exc=%0b%0b%0b%0b%0b for expected %s : %s",
                                     i, o.dz[i], o.ov[i], o.un[i], o.sinv[i], o.exc[i], ref_exc_name(e.exc[i]), ctx));
        end
      end
      else begin
        if (o.code[i] !== 3'd0 || o.dz[i] !== 1'b0 || o.ov[i] !== 1'b0 ||
            o.un[i] !== 1'b0 || o.sinv[i] !== 1'b0 || o.exc[i] !== 1'b0) begin
          e_exc++;
          flag("SB_MASKED_EXC", $sformatf("masked lane%0d raised exception code=%0d flags=%0b%0b%0b%0b%0b : %s",
                                          i, o.code[i], o.dz[i], o.ov[i], o.un[i], o.sinv[i], o.exc[i], ctx));
        end
      end
    end

    if (is_reduction(t.opcode)) begin
      if (o.red_valid !== 1'b1) begin
        e_red++;
        flag("SB_RED_VALID", $sformatf("reduction_valid_out low for %s", ctx));
      end
      case (t.opcode)
        OPC_DOT: if (o.dot !== e.dot) begin
          e_red++;
          flag("SB_DOT", $sformatf("dot exp=%05h got=%05h : %s", e.dot, o.dot, ctx));
        end
        OPC_MAC: if (o.mac !== e.mac) begin
          e_red++;
          flag("SB_MAC", $sformatf("mac exp=%05h got=%05h : %s", e.mac, o.mac, ctx));
        end
        OPC_SAD: if (o.sad !== e.sad) begin
          e_red++;
          flag("SB_SAD", $sformatf("sad exp=%05h got=%05h : %s", e.sad, o.sad, ctx));
        end
        default: ;
      endcase
    end
    else if (o.red_valid !== 1'b0) begin
      e_red++;
      flag("SB_RED_VALID", $sformatf("reduction_valid_out high for non-reduction op : %s", ctx));
    end
  endfunction

  // ----------------------------------------------------------------- phases
  function void check_phase(uvm_phase phase);
    super.check_phase(phase);
    if (q.size() != 0) begin
      `uvm_error("SB_PENDING", $sformatf("%0d accepted transactions never completed", q.size()))
    end
    if (total_errors() != 0) begin
      `uvm_error("SB_SUMMARY", $sformatf("%0d mismatches in total (only the first %0d were printed)",
                                         total_errors(), MAX_PRINT))
    end
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("SB", $sformatf({"\n=========== SCOREBOARD SUMMARY ===========\n",
        " accepted %0d  compared %0d  flushed-by-reset %0d  pending %0d\n",
        " errors: latency %0d  ctrl %0d  mask %0d  data %0d  exception %0d  reduction %0d  misc %0d\n",
        "=========================================="},
        n_in, n_out, n_flushed, q.size(), e_lat, e_ctrl, e_mask, e_data, e_exc, e_red, e_misc), UVM_NONE)
    // Guard against a vacuous pass: a test that compares nothing must not pass.
    if (n_out == 0) begin
      `uvm_error("SB", "No transactions were compared - vacuous pass")
    end
  endfunction
endclass
