/**
 *
 * Name:
 *   bp_me_bedrock_register.sv
 *
 * Description:
 *   This module is used to interface a BP Stream interface to a general-purpose
 *   register read/write interface. The data is stored externally so that
 *   control/status registers can be controlled by this interface while
 *   retaining special semantics. Registers are assumed to be synchronous
 *   read/write which is compatible (although suboptimal) for asynchronous
 *   registers.
 *
 */

`include "bp_common_defines.svh"
`include "bp_me_defines.svh"

module bp_me_bedrock_register
 import bp_common_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)

   // The width of the registers. Currently, must all be the same.
   , parameter reg_width_p = dword_width_gp
   // The address width of the registers. For addresses less than paddr_width_p,
   //   the upper bits of the paddr are ignored for matching purposes
   , parameter reg_addr_width_p = paddr_width_p
   // The number of registers to control
   , parameter els_p = 1

   // We would like to use unpacked here, but Verilator 4.202 does not support it
   // Unsupported tristate construct: INITITEM
   //// An unpacked array of integer register base addresses
   ////   e.g. localparam integer base_addr_lp [1:0] = '{0xf00bad, 0x00cafe}
   //// Can also accept pattern matches such as 0x8???
   //, parameter integer base_addr_p [els_p-1:0] = '{0}
   , parameter [els_p-1:0][reg_addr_width_p-1:0] base_addr_p = '0

   , localparam lg_reg_width_lp = `BSG_WIDTH(`BSG_SAFE_CLOG2(reg_width_p/8))
   )
  (input                                            clk_i
   , input                                          reset_i

   // Network-side BP-Stream interface
   , input [mem_header_width_lp-1:0]                mem_cmd_header_i
   , input                                          mem_cmd_header_v_i
   , output logic                                   mem_cmd_header_ready_and_o
   , input                                          mem_cmd_has_data_i
   , input [dword_width_gp-1:0]                     mem_cmd_data_i
   , input                                          mem_cmd_data_v_i
   , output logic                                   mem_cmd_data_ready_and_o
   , input                                          mem_cmd_last_i

   , output logic [mem_header_width_lp-1:0]         mem_resp_header_o
   , output logic                                   mem_resp_header_v_o
   , input                                          mem_resp_header_ready_and_i
   , output logic                                   mem_resp_has_data_o
   , output logic [dword_width_gp-1:0]              mem_resp_data_o
   , output logic                                   mem_resp_data_v_o
   , input                                          mem_resp_data_ready_and_i
   , output logic                                   mem_resp_last_o

   // Synchronous register read/write interface.
   // Actually 1rw, but expose both ports to prevent unnecessary and gates
   // Assume latch last read behavior at registers, and do not have
   //   unnecessary read/writes. This could be parameterizable, but requires
   //   a read register in this module to do and maintain helpfulness
   , output logic [els_p-1:0]                       r_v_o
   , output logic [els_p-1:0]                       w_v_o
   , output logic [reg_addr_width_p-1:0]            addr_o
   , output logic [lg_reg_width_lp-1:0]             size_o
   , output logic [reg_width_p-1:0]                 data_o
   , input [els_p-1:0][reg_width_p-1:0]             data_i
   );

  `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);
  `bp_cast_i(bp_bedrock_mem_header_s, mem_cmd_header);
  `bp_cast_o(bp_bedrock_mem_header_s, mem_resp_header);

  enum logic [1:0] {e_ready, e_read_rx, e_write_tx} state_n, state_r;
  wire is_ready    = (state_r == e_ready);
  wire is_read_rx  = (state_r == e_read_rx);
  wire is_write_tx = (state_r == e_write_tx);

  bp_bedrock_mem_header_s mem_cmd_header_li;
  logic mem_cmd_header_v_li, mem_cmd_header_yumi_lo, mem_cmd_has_data_li;
  bsg_one_fifo
   #(.width_p(1+$bits(bp_bedrock_mem_header_s)))
   cmd_header_fifo
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({mem_cmd_has_data_i, mem_cmd_header_i})
     ,.v_i(mem_cmd_header_v_i)
     ,.ready_o(mem_cmd_header_ready_and_o)

     ,.data_o({mem_cmd_has_data_li, mem_cmd_header_li})
     ,.v_o(mem_cmd_header_v_li)
     ,.yumi_i(mem_cmd_header_yumi_lo)
     );

  assign mem_resp_header_cast_o = mem_cmd_header_li;
  assign mem_resp_header_v_o = mem_cmd_header_v_li & (is_write_tx | is_read_rx);
  assign mem_resp_has_data_o = ~mem_cmd_has_data_i;
  assign mem_cmd_header_yumi_li = mem_resp_header_ready_and_i & mem_resp_header_v_o;

  logic [dword_width_gp-1:0] mem_cmd_data_li;
  logic mem_cmd_data_v_li, mem_cmd_data_yumi_lo, mem_cmd_last_li;
  bsg_one_fifo
   #(.width_p(1+dword_width_gp))
   cmd_data_fifo
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({mem_cmd_last_i, mem_cmd_data_i})
     ,.v_i(mem_cmd_data_v_i)
     ,.ready_o(mem_cmd_data_ready_and_o)

     ,.data_o({mem_cmd_last_li, mem_cmd_data_li})
     ,.v_o(mem_cmd_data_v_li)
     ,.yumi_i(mem_cmd_data_yumi_lo)
     );

  logic [els_p-1:0] r_v_r;
  bsg_dff
   #(.width_p(els_p))
   r_v_reg
    (.clk_i(clk_i)
     ,.data_i(r_v_o)
     ,.data_o(r_v_r)
     );

  logic [reg_width_p-1:0] rdata_lo;
  bsg_mux_one_hot
   #(.width_p(reg_width_p), .els_p(els_p))
   rmux_oh
    (.data_i(data_i)
     ,.sel_one_hot_i(r_v_r)
     ,.data_o(rdata_lo)
     );

  logic rdata_ready_lo;
  bsg_one_fifo
   #(.width_p(reg_width_p))
   rdata_fifo
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i(rdata_lo)
     ,.v_i(|r_v_r)
     ,.ready_o(rdata_ready_lo)

     ,.data_o(mem_resp_data_o)
     ,.v_o(mem_resp_data_v_o)
     ,.yumi_i(mem_resp_data_ready_and_i & mem_resp_data_v_o)
     );
  assign mem_resp_last_o = mem_resp_data_v_o;

  logic [els_p-1:0][reg_addr_width_p-1:0] addr_match;
  wire wr_not_rd = (mem_cmd_header_li.msg_type inside {e_bedrock_mem_wr, e_bedrock_mem_uc_wr});
  wire rd_not_wr = (mem_cmd_header_li.msg_type inside {e_bedrock_mem_rd, e_bedrock_mem_uc_rd});
  wire [reg_addr_width_p-1:0] reg_addr_li = mem_cmd_header_li.addr[0+:reg_addr_width_p];
  for (genvar i = 0; i < els_p; i++)
    begin
      assign addr_match[i] = (reg_addr_li inside {base_addr_p[i]});
      assign r_v_o[i] = is_ready & mem_cmd_header_v_li & addr_match & ~wr_not_rd;
      assign w_v_o[i] = is_ready & mem_cmd_header_v_li & addr_match &  wr_not_rd;
    end

  always_comb
    begin
      case (state_r)
        e_ready   : state_n = |r_v_o ? e_read_rx : |w_v_o ? e_write_tx : e_ready;
        e_write_tx: state_n = (mem_resp_header_ready_and_i & mem_resp_header_v_o) ? e_ready : e_write_tx;
        e_read_rx : state_n = (~mem_cmd_header_v_li & mem_resp_data_ready_and_i & mem_resp_data_v_o) ? e_ready : e_read_rx;
        default : state_n = state_r;
      endcase
    end

  // synopsys sync_set_reset "reset_i"
  always_ff @(posedge clk_i)
    if (reset_i)
      state_r <= e_ready;
    else
      state_r <= state_n;

  assign addr_o = mem_cmd_header_li.addr[0+:reg_addr_width_p];
  assign size_o = mem_cmd_header_li.size;
  assign data_o = mem_cmd_data_li;

  if (dword_width_gp != 64) $error("BedRock interface data width must be 64-bits");

  //synopsys translate_off
  always_ff @(negedge clk_i)
    begin
      //assert(reset_i !== '0 || ~mem_cmd_header_v_li | (v_r | ~wr_not_rd | |w_v_o) | (v_r | ~rd_not_wr | |r_v_o))
      //  else $error("Command to non-existent register: %x", addr_o);

      //assert(reset_i !== '0 || ~(mem_cmd_data_v_i & mem_cmd_data_ready_and_o) || mem_cmd_data_last_i)
      //  else $error("Multi-beat memory command detected");
    end
  //synopsys translate_on

endmodule

