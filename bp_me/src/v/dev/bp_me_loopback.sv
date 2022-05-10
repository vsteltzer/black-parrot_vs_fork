/**
 *
 * Name:
 *   bp_me_loopback.sv
 *
 * Description:
 *   This module is an active tie-off. That is, requests to this module will return the header
 *   with a zero payload. This is useful to not stall the network in the case of an erroneous
 *   address, or prevent deadlock at network boundaries
 *
 */
`include "bp_common_defines.svh"
`include "bp_me_defines.svh"

module bp_me_loopback
 import bp_common_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)
   )
  (input                                            clk_i
   , input                                          reset_i

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
   );

  `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);

  bp_bedrock_mem_header_s mem_cmd_header_li;
  logic mem_cmd_header_v_li, mem_cmd_header_yumi_lo, mem_cmd_has_data_li;
  bsg_one_fifo
   #(.width_p(1+mem_header_width_lp))
   header_loopback_buffer
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({mem_cmd_has_data_i, mem_cmd_header_i})
     ,.v_i(mem_cmd_header_v_i)
     ,.ready_o(mem_cmd_header_ready_and_o)

     ,.data_o({mem_cmd_has_data_li, mem_cmd_header_li})
     ,.v_o(mem_cmd_header_v_li)
     ,.yumi_i(mem_cmd_header_yumi_lo)
     );

  logic [dword_width_gp-1:0] mem_cmd_data_li;
  logic mem_cmd_data_v_li, mem_cmd_data_yumi_lo, mem_cmd_last_li;
  bsg_one_fifo
   #(.width_p(1+dword_width_gp))
   data_loopback_buffer
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.data_i({mem_cmd_has_data_i, mem_cmd_header_i})
     ,.v_i(mem_cmd_header_v_i)
     ,.ready_o(mem_cmd_header_ready_and_o)

     ,.data_o({mem_cmd_last_li, mem_cmd_data_li})
     ,.v_o(mem_cmd_data_v_li)
     ,.yumi_i(mem_cmd_data_yumi_lo)
     );

  assign mem_resp_header_o = mem_cmd_header_li;
  assign mem_resp_header_v_o = mem_cmd_header_v_li;
  assign mem_resp_has_data_o = ~mem_cmd_has_data_li;
  assign mem_cmd_header_yumi_lo = mem_resp_header_ready_and_i & mem_resp_header_v_o;

  assign mem_resp_data_o = mem_cmd_data_li;
  assign mem_resp_data_v_o = mem_cmd_data_v_li;
  assign mem_resp_last_o = mem_cmd_last_li;
  assign mem_cmd_data_yumi_lo = mem_resp_data_ready_and_i & mem_resp_data_v_o;

endmodule

