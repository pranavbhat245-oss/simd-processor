// ---------------------------------------------------------------------------
// Functional coverage WITHOUT covergroups (Questa Starter has no license for
// them). Same idea, done by hand: count hits per bin in arrays, then report
// the percentage and the holes at the end of the run.
// ---------------------------------------------------------------------------
class simd_coverage extends uvm_component;
  `uvm_component_utils(simd_coverage)

  uvm_analysis_imp_in  #(simd_txn,     simd_coverage) in_imp;
  uvm_analysis_imp_out #(simd_out_txn, simd_coverage) out_imp;

  int unsigned cnt_op_sm   [16][2];        // opcode x signed
  int unsigned cnt_mask    [16];           // lane mask (1..15)
  int unsigned cnt_op_exc  [16][5];        // opcode x exception code (output side)
  int unsigned cnt_op_a_b  [16][6][6];     // opcode x class(a) x class(b), per active lane
  int unsigned cnt_red     [3][2][2];      // {DOT,MAC,SAD} x signed x relu_en

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_imp  = new("in_imp",  this);
    out_imp = new("out_imp", this);
  endfunction

  // operand class: 0:0x00 1:0x01 2:0x7F 3:0x80 4:0xFF 5:other
  static function int cls(bit [7:0] v);
    case (v)
      8'h00:   return 0;
      8'h01:   return 1;
      8'h7F:   return 2;
      8'h80:   return 3;
      8'hFF:   return 4;
      default: return 5;
    endcase
  endfunction

  // which exceptions can an opcode raise at all (impossible ones are not holes)
  static function bit exc_possible(int op, int x);
    case (x)
      0:       return 1;
      1:       return (op == OPC_DIV);
      2:       return (op == OPC_ADD) || (op == OPC_SUB) || (op == OPC_MULT) || (op == OPC_SATADD);
      3:       return (op == OPC_SUB);
      4:       return (op == OPC_SQRT);
      default: return 0;
    endcase
  endfunction

  function void write_in(simd_txn t);
    int ri;
    cnt_op_sm[t.opcode][t.signed_mode]++;
    cnt_mask[t.mask]++;
    for (int i = 0; i < 4; i++)
      if (t.mask[i]) cnt_op_a_b[t.opcode][cls(t.a[i])][cls(t.b[i])]++;
    ri = (t.opcode == OPC_DOT) ? 0 : (t.opcode == OPC_MAC) ? 1 : (t.opcode == OPC_SAD) ? 2 : -1;
    if (ri >= 0) cnt_red[ri][t.signed_mode][t.relu_en]++;
  endfunction

  function void write_out(simd_out_txn o);
    for (int i = 0; i < 4; i++)
      if (o.lane_valid[i] === 1'b1 && !$isunknown(o.code[i]) && o.code[i] <= 3'd4)
        cnt_op_exc[o.opcode][o.code[i]]++;
  endfunction

  function void report_phase(uvm_phase phase);
    int unsigned cov, tot, g_cov, g_tot;
    string       holes, line;
    super.report_phase(phase);
    cov = 0;  tot = 0;
    line = "\n=========== FUNCTIONAL COVERAGE (hand-made bins) ===========\n";

    // opcode x signed
    g_cov = 0;  g_tot = 0;  holes = "";
    for (int o = 0; o < 16; o++)
      for (int s = 0; s < 2; s++) begin
        g_tot++;
        if (cnt_op_sm[o][s] != 0) g_cov++;
        else holes = {holes, $sformatf(" %s/%s", ref_op_name(o), s ? "signed" : "unsigned")};
      end
    line = {line, $sformatf(" opcode x signed     : %0d/%0d  holes:%s\n", g_cov, g_tot, (holes == "") ? " none" : holes)};
    cov += g_cov;  tot += g_tot;

    // lane mask
    g_cov = 0;  g_tot = 0;  holes = "";
    for (int m = 1; m < 16; m++) begin
      g_tot++;
      if (cnt_mask[m] != 0) g_cov++;
      else holes = {holes, $sformatf(" %04b", m[3:0])};
    end
    line = {line, $sformatf(" lane mask           : %0d/%0d  holes:%s\n", g_cov, g_tot, (holes == "") ? " none" : holes)};
    cov += g_cov;  tot += g_tot;

    // opcode x exception (legal combinations only)
    g_cov = 0;  g_tot = 0;  holes = "";
    for (int o = 0; o < 16; o++)
      for (int x = 0; x < 5; x++)
        if (exc_possible(o, x)) begin
          g_tot++;
          if (cnt_op_exc[o][x] != 0) g_cov++;
          else holes = {holes, $sformatf(" %s/%s", ref_op_name(o), ref_exc_name(x[2:0]))};
        end
    line = {line, $sformatf(" opcode x exception  : %0d/%0d  holes:%s\n", g_cov, g_tot, (holes == "") ? " none" : holes)};
    cov += g_cov;  tot += g_tot;

    // DOT/MAC/SAD x signed x relu
    g_cov = 0;  g_tot = 0;
    for (int r = 0; r < 3; r++)
      for (int s = 0; s < 2; s++)
        for (int l = 0; l < 2; l++) begin
          g_tot++;
          if (cnt_red[r][s][l] != 0) g_cov++;
        end
    line = {line, $sformatf(" reduction x sm x relu: %0d/%0d\n", g_cov, g_tot)};
    cov += g_cov;  tot += g_tot;

    // opcode x class(a) x class(b)
    g_cov = 0;  g_tot = 0;
    for (int o = 0; o < 16; o++)
      for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++) begin
          g_tot++;
          if (cnt_op_a_b[o][x][y] != 0) g_cov++;
        end
    line = {line, $sformatf(" opcode x a-class x b-class: %0d/%0d\n", g_cov, g_tot)};
    cov += g_cov;  tot += g_tot;

    line = {line, $sformatf(" TOTAL: %0d/%0d bins = %0.1f%%\n", cov, tot, 100.0 * cov / tot),
            "============================================================"};
    `uvm_info("COV", line, UVM_NONE)
  endfunction
endclass
