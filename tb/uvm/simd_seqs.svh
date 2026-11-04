// ---------------------------------------------------------------------------
// Sequences
// ---------------------------------------------------------------------------
class simd_base_seq extends uvm_sequence #(simd_txn);
  `uvm_object_utils(simd_base_seq)

  function new(string name = "simd_base_seq");
    super.new(name);
  endfunction

  // Send one fully-specified (non-random) transaction.
  task send_fixed(bit [3:0] op, bit sm, bit rl, bit [3:0] m,
                  byte_arr_t aa, byte_arr_t bb, bit [15:0] cc, int unsigned g = 0);
    simd_txn t;
    t = simd_txn::type_id::create("t");
    start_item(t);
    t.opcode = op;  t.signed_mode = sm;  t.relu_en = rl;  t.mask = m;
    t.a = aa;  t.b = bb;  t.c = cc;  t.gap = g;
    finish_item(t);
  endtask
endclass

// Random traffic ($urandom based). op_en[i]=1 allows opcode i.
class simd_rand_seq extends simd_base_seq;
  `uvm_object_utils(simd_rand_seq)
  int unsigned n      = 100;
  bit [15:0]   op_en  = 16'hFFFF;
  bit          no_gap = 0;

  function new(string name = "simd_rand_seq");
    super.new(name);
  endfunction

  task body();
    simd_txn t;
    repeat (n) begin
      t = simd_txn::type_id::create("t");
      start_item(t);
      t.pseudo_rand(op_en, -1, -1, -1, no_gap);
      finish_item(t);
    end
  endtask
endclass

// Every opcode, both modes, a grid of corner-value operand pairs.
class simd_corner_seq extends simd_base_seq;
  `uvm_object_utils(simd_corner_seq)

  function new(string name = "simd_corner_seq");
    super.new(name);
  endfunction

  task body();
    byte_arr_t aa, bb;
    // 12 interesting operand values, see corner()
    for (int op = 0; op < 16; op++)
      for (int sm = 0; sm < 2; sm++)
        for (int k = 0; k < 144; k++) begin
          for (int i = 0; i < 4; i++) begin
            aa[i] = corner((k + i) % 12);
            bb[i] = corner(((k / 12) + i * 5) % 12);
          end
          send_fixed(op[3:0], sm[0], k[0], 4'hF, aa, bb, k[0] ? 16'hFFF0 : 16'h0010);
        end
    // targeted exception corners (all lanes)
    send_fixed(OPC_DIV,  1, 0, 4'hF, '{8'h80, 8'h80, 8'h7F, 8'h01}, '{8'hFF, 8'h00, 8'h00, 8'hFF}, 0);
    send_fixed(OPC_DIV,  0, 0, 4'hF, '{8'd10, 8'd20, 8'd30, 8'd40}, '{8'h00, 8'h00, 8'h00, 8'h00}, 0);
    send_fixed(OPC_SQRT, 1, 0, 4'hF, '{8'h81, 8'h7F, 8'hF0, 8'h10}, '{8'h00, 8'h00, 8'h00, 8'h00}, 0);
    send_fixed(OPC_MAC,  1, 0, 4'hF, '{8'h80, 8'h80, 8'h80, 8'h80}, '{8'h7F, 8'h7F, 8'h7F, 8'h7F}, 16'h8000);
    send_fixed(OPC_MAC,  0, 0, 4'hF, '{8'hFF, 8'hFF, 8'hFF, 8'hFF}, '{8'hFF, 8'hFF, 8'hFF, 8'hFF}, 16'hFFFF);
  endtask

  function automatic bit [7:0] corner(int idx);
    case (idx)
      0: return 8'h00;  1: return 8'h01;  2: return 8'h02;  3: return 8'h0F;
      4: return 8'h55;  5: return 8'h7F;  6: return 8'h80;  7: return 8'h81;
      8: return 8'hAA;  9: return 8'hF0; 10: return 8'hFE; default: return 8'hFF;
    endcase
  endfunction
endclass

// Every non-zero lane mask for a set of opcodes (both modes).
class simd_mask_seq extends simd_base_seq;
  `uvm_object_utils(simd_mask_seq)
  int unsigned reps = 4;

  function new(string name = "simd_mask_seq");
    super.new(name);
  endfunction

  task body();
    simd_txn t;
    bit [3:0] ops [6];
    ops = '{OPC_ADD, OPC_DIV, OPC_SQRT, OPC_DOT, OPC_MAC, OPC_SAD};
    for (int m = 1; m < 16; m++)
      foreach (ops[o])
        for (int sm = 0; sm < 2; sm++)
          repeat (reps) begin
            t = simd_txn::type_id::create("t");
            start_item(t);
            t.pseudo_rand(16'hFFFF, ops[o], sm, m, 1'b1);
            finish_item(t);
          end
  endtask
endclass

// All 65536 (a,b) pairs of one opcode/mode, four pairs per transaction.
class simd_sweep_seq extends simd_base_seq;
  `uvm_object_utils(simd_sweep_seq)
  bit [3:0] op    = OPC_ADD;
  bit       sm    = 0;
  bit       unary = 0;     // 1: only 'a' matters, sweep 256 values of a

  function new(string name = "simd_sweep_seq");
    super.new(name);
  endfunction

  task body();
    byte_arr_t aa, bb;
    if (unary) begin
      for (int ia = 0; ia < 256; ia += 4) begin
        for (int i = 0; i < 4; i++) begin aa[i] = ia + i; bb[i] = 8'h55; end
        send_fixed(op, sm, 1'b0, 4'hF, aa, bb, 16'h0);
      end
    end
    else begin
      for (int ia = 0; ia < 256; ia++)
        for (int ib = 0; ib < 256; ib += 4) begin
          for (int i = 0; i < 4; i++) begin aa[i] = ia + i; bb[i] = ib + i; end
          send_fixed(op, sm, ia[0], 4'hF, aa, bb, 16'(ia * 257));
        end
    end
  endtask
endclass

// mask == 0 : documents that the DUT never accepts it (ready_in stays low).
class simd_zero_mask_seq extends simd_base_seq;
  `uvm_object_utils(simd_zero_mask_seq)

  function new(string name = "simd_zero_mask_seq");
    super.new(name);
  endfunction

  task body();
    byte_arr_t z;
    z = '{8'h01, 8'h02, 8'h03, 8'h04};
    repeat (3) send_fixed(OPC_ADD, 0, 0, 4'b0000, z, z, 0);
  endtask
endclass
