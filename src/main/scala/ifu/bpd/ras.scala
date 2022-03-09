//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.ifu

import chisel3._
import chisel3.util._
import chisel3.core.{withReset}
import chisel3.internal.sourceinfo.{SourceInfo}

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.diplomaticobjectmodel.logicaltree.{ICacheLogicalTreeNode}

import boom.common._
import boom.exu.{CommitExceptionSignals, BranchDecode, BrUpdateInfo}
import boom.util._

class BoomRAS(implicit p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val read_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val read_addr  = Output(UInt(vaddrBitsExtended.W))

    val write_valid = Input(Bool())
    val write_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val write_addr  = Input(UInt(vaddrBitsExtended.W))

    // For update read_pred_set
    val read_pred_set_valid = Output(Bool())
    val read_pred_set = Output(UInt(predSetBits.W))
    
    val write_pred_set_valid = Input(Bool())
    val write_pred_set  = Input(UInt(predSetBits.W))
  })

  class BoomRASPred extends Bundle {
    val pred_set = UInt(predSetBits.W)
    val valid = Bool()
  }

  val ras = Reg(Vec(nRasEntries, UInt(vaddrBitsExtended.W)))
  val ras_pred = Reg(Vec(nRasEntries, new BoomRASPred))

  when (RegNext(io.write_valid && io.write_idx === io.read_idx)) {
    io.read_addr := RegNext(io.write_addr)
    io.read_pred_set := RegNext(io.write_pred_set)
    io.read_pred_set_valid := RegNext(io.write_pred_set_valid)
  }
  .otherwise {
    io.read_addr := RegNext(ras(io.read_idx))
    io.read_pred_set := RegNext(ras_pred(io.read_idx).pred_set)
    io.read_pred_set_valid := RegNext(ras_pred(io.read_idx).valid)
  }

  when (io.write_valid) {
    ras(io.write_idx) := io.write_addr
    when (io.write_pred_set_valid) {
      ras_pred(io.write_idx).pred_set := io.write_pred_set
      ras_pred(io.write_idx).valid := true.B
    }
    .otherwise {
      ras_pred(io.write_idx).valid := false.B
    }
  }
}
