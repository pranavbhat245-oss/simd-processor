// ---------------------------------------------------------------------------
// Input transaction (stimulus, and also what the monitor reports on accept)
// ---------------------------------------------------------------------------
class simd_txn extends uvm_sequence_item;
  // NOTE: no 'rand' / constraints / randomize() on purpose. Questa Starter Edition
  // has no license for constrained-random, so randomisation is done by hand with
  // $urandom in pseudo_rand() below.
  bit [3:0]        opcode;
  bit              signed_mode;
  bit              relu_en;
  bit [3:0]        mask;
  byte_arr_t       a;
  byte_arr_t       b;
  bit [15:0]       c;
  int unsigned     gap;            // idle clocks inserted before this transaction

  bit              use_corners = 1; // bias operands toward 0/1/0x7F/0x80/0xFF

  // filled in by the monitor
  longint unsigned acc_cycle;
  longint unsigned acc_stall;

  `uvm_object_utils(simd_txn)

  function new(string name = "simd_txn");
    super.new(name);
  endfunction

  // Weighted byte: 00(3) 01(2) 7F(3) 80(3) FF(3) FE(1) 81(1) low-range(20) high-range(20)
  static function bit [7:0] pick_byte(bit corners);
    int r;
    if (!corners) return 8'($urandom());
    r = $urandom_range(55, 0);
    if      (r <=  2) return 8'h00;
    else if (r <=  4) return 8'h01;
    else if (r <=  7) return 8'h7F;
    else if (r <= 10) return 8'h80;
    else if (r <= 13) return 8'hFF;
    else if (r == 14) return 8'hFE;
    else if (r == 15) return 8'h81;
    else if (r <= 35) return 8'($urandom_range(8'h7E, 8'h02));
    else              return 8'($urandom_range(8'hFD, 8'h82));
  endfunction

  // Fill every field with (weighted) random values.
  //   op_en       bit i set  => opcode i may be chosen
  //   force_*     >= 0       => use this value instead of a random one
  //   no_gap      1          => gap = 0
  function void pseudo_rand(bit [15:0] op_en = 16'hFFFF, int force_op = -1,
                            int force_sm = -1, int force_mask = -1, bit no_gap = 0);
    int r;
    if (force_op >= 0) opcode = force_op[3:0];
    else begin
      do opcode = 4'($urandom_range(15, 0)); while (!op_en[opcode]);
    end
    signed_mode = (force_sm   >= 0) ? force_sm[0]   : 1'($urandom_range(1, 0));
    mask        = (force_mask >= 0) ? force_mask[3:0] : 4'($urandom_range(15, 1));
    relu_en     = 1'($urandom_range(1, 0));
    for (int i = 0; i < 4; i++) begin
      a[i] = pick_byte(use_corners);
      b[i] = pick_byte(use_corners);
    end
    r = $urandom_range(16, 0);
    if (!use_corners || r > 6) c = 16'($urandom());
    else if (r <= 2)           c = 16'h0000;
    else if (r <= 4)           c = 16'hFFFF;
    else if (r == 5)           c = 16'h8000;
    else                       c = 16'h7FFF;
    r = $urandom_range(99, 0);
    if (no_gap || r < 70) gap = 0;
    else if (r < 95)      gap = $urandom_range(3, 1);
    else                  gap = $urandom_range(12, 4);
  endfunction

  virtual function string convert2string();
    return $sformatf("op=%s sm=%0b relu=%0b mask=%04b a={%02h,%02h,%02h,%02h} b={%02h,%02h,%02h,%02h} c=%04h gap=%0d",
                     ref_op_name(opcode), signed_mode, relu_en, mask,
                     a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3], c, gap);
  endfunction
endclass

// ---------------------------------------------------------------------------
// Output transaction (what the DUT produced when valid_out && ready_out)
// 4-state 'logic' fields on purpose, so X on an output is never hidden.
// ---------------------------------------------------------------------------
class simd_out_txn extends uvm_sequence_item;
  logic [15:0]     lane_res [4];
  logic [3:0]      lane_valid;
  logic [18:0]     dot, mac, sad;
  logic            red_valid;
  logic [3:0]      opcode;
  logic            signed_mode;
  logic [3:0]      dz, ov, un, sinv, exc;
  logic [2:0]      code [4];
  longint unsigned done_cycle;
  longint unsigned done_stall;

  `uvm_object_utils(simd_out_txn)

  function new(string name = "simd_out_txn");
    super.new(name);
  endfunction

  virtual function string convert2string();
    return $sformatf("op=%s lanes={%04h,%04h,%04h,%04h} lane_valid=%04b red_valid=%0b dot=%05h mac=%05h sad=%05h",
                     ref_op_name(opcode), lane_res[0], lane_res[1], lane_res[2], lane_res[3],
                     lane_valid, red_valid, dot, mac, sad);
  endfunction
endclass
