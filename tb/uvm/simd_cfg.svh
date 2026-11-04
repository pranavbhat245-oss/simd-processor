class simd_cfg extends uvm_object;
  `uvm_object_utils(simd_cfg)

  // Backpressure on ready_out:  0 = always ready, 1 = ready 75%, 2 = ready 25%, 3 = random bursts
  int unsigned ready_mode = 0;
  // Driver gives up (fatal) if a transaction is not accepted within this many clocks
  int unsigned max_wait   = 5000;

  function new(string name = "simd_cfg");
    super.new(name);
  endfunction
endclass
