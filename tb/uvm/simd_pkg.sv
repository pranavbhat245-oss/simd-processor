`timescale 1ns/1ps
package simd_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import simd_ref_pkg::*;

  `include "simd_decls.svh"
  `include "simd_cfg.svh"
  `include "simd_txn.svh"
  `include "simd_seqs.svh"
  `include "simd_driver.svh"
  `include "simd_monitor.svh"
  `include "simd_scoreboard.svh"
  `include "simd_coverage.svh"
  `include "simd_agent_env.svh"
  `include "simd_tests.svh"
endpackage
