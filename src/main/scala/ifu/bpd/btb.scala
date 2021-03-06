package boom.ifu

import chisel3._
import chisel3.experimental.BundleLiterals._
import chisel3.util._

import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.common._
import boom.util.{BoomCoreStringPrefix}

import scala.math.min

case class BoomBTBParams(
  nSets: Int = 128,
  nWays: Int = 2,
  offsetSz: Int = 13,
  extendedNSets: Int = 128
)


class BTBBranchPredictorBank(params: BoomBTBParams = BoomBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  override val nSets         = params.nSets
  override val nWays         = params.nWays
  val tagSz         = vaddrBitsExtended - log2Ceil(nSets) - log2Ceil(fetchWidth) - 1
  val offsetSz      = params.offsetSz
  val extendedNSets = params.extendedNSets

  require(isPow2(nSets))
  require(isPow2(extendedNSets) || extendedNSets == 0)
  require(extendedNSets <= nSets)
  require(extendedNSets >= 1)

  class BTBEntry extends Bundle {
    val offset   = SInt(offsetSz.W)
    val extended = Bool()
  }
  val btbEntrySz = offsetSz + 1

  class BTBMeta extends Bundle {
    val is_br = Bool()
    val tag   = UInt(tagSz.W)
  }
  val btbMetaSz = tagSz + 1

  class BTBPredictMeta extends Bundle {
    val write_way = UInt(log2Ceil(nWays).W)
  }

  // Update BTB entry when branch jump completes
  class BTBIndexInfo extends Bundle{
    val pc = UInt(vaddrBitsExtended.W)
    val way = UInt(log2Ceil(nWays).W)
    val valid = Bool()
  }

  // Info about pred set index
  class BTBPredSetIndex extends Bundle{
    val pred_set = UInt(predSetBits.W)
    val valid = Bool()
  }
  val btbpredSetIndexSz = 3

  val s1_meta = Wire(new BTBPredictMeta)
  val f3_meta = RegNext(RegNext(s1_meta))


  io.f3_meta := f3_meta.asUInt

  override val metaSz = s1_meta.asUInt.getWidth

  val doing_reset = RegInit(true.B)
  val reset_idx   = RegInit(0.U(log2Ceil(nSets).W))
  reset_idx := reset_idx + doing_reset
  when (reset_idx === (nSets-1).U) { doing_reset := false.B }

  val meta     = Seq.fill(nWays) { SyncReadMem(nSets, Vec(bankWidth, UInt(btbMetaSz.W))) }
  val btb      = Seq.fill(nWays) { SyncReadMem(nSets, Vec(bankWidth, UInt(btbEntrySz.W))) }
  val ebtb     = SyncReadMem(extendedNSets, UInt(vaddrBitsExtended.W))
  
  // Info about pred_set_index
  val pset = Seq.fill(nWays) { SyncReadMem(nSets, Vec(bankWidth, UInt(btbpredSetIndexSz.W))) }
  val s1_btb_hit_index = RegInit((0.U).asTypeOf(new BTBIndexInfo()))
  val s2_btb_hit_index = RegNext(s1_btb_hit_index)
  val s3_btb_hit_index = RegNext(s2_btb_hit_index)

  val mems = (((0 until nWays) map ({w:Int => Seq(
    (f"btb_meta_way$w", nSets, bankWidth * btbMetaSz),
    (f"btb_data_way$w", nSets, bankWidth * btbEntrySz))})).flatten ++ Seq(("ebtb", extendedNSets, vaddrBitsExtended)))

  val s1_req_rbtb  = VecInit(btb.map { b => VecInit(b.read(s0_idx , s0_valid).map(_.asTypeOf(new BTBEntry))) })
  val s1_req_rmeta = VecInit(meta.map { m => VecInit(m.read(s0_idx, s0_valid).map(_.asTypeOf(new BTBMeta))) })
  val s1_req_rebtb = ebtb.read(s0_idx, s0_valid)
  val s1_req_tag   = s1_idx >> log2Ceil(nSets)

  val s1_req_rpred = VecInit(pset.map { p => VecInit(p.read(s0_idx, s0_valid).map(_.asTypeOf(new BTBPredSetIndex))) })

  val s1_resp   = Wire(Vec(bankWidth, Valid(UInt(vaddrBitsExtended.W))))
  val s1_is_br  = Wire(Vec(bankWidth, Bool()))
  val s1_is_jal = Wire(Vec(bankWidth, Bool()))

  val s1_hit_ohs = VecInit((0 until bankWidth) map { i =>
    VecInit((0 until nWays) map { w =>
      s1_req_rmeta(w)(i).tag === s1_req_tag(tagSz-1,0)
    })
  })
  val s1_hits     = s1_hit_ohs.map { oh => oh.reduce(_||_) }
  // Returns the bit position of the least-significant high bit of the input bitvector.
  val s1_hit_ways = s1_hit_ohs.map { oh => PriorityEncoder(oh) }

  // Debug info
  val debug_flag = false.B
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)

  for (w <- 0 until bankWidth) {
    val entry_meta = s1_req_rmeta(s1_hit_ways(w))(w)
    val entry_btb  = s1_req_rbtb(s1_hit_ways(w))(w)
    val entry_pred_set_valid = s1_req_rpred(s1_hit_ways(w))(w).valid
    val entry_pred_set = s1_req_rpred(s1_hit_ways(w))(w).pred_set

    s1_resp(w).valid := !doing_reset && s1_valid && s1_hits(w)
    s1_resp(w).bits  := Mux(
      entry_btb.extended,
      s1_req_rebtb,
      (s1_pc.asSInt + (w << 1).S + entry_btb.offset).asUInt)
    s1_is_br(w)  := !doing_reset && s1_resp(w).valid &&  entry_meta.is_br
    s1_is_jal(w) := !doing_reset && s1_resp(w).valid && !entry_meta.is_br

    when (debug_flag && s1_hits(w)) {
      printf("btb f1, cycle: %d", debug_cycles.value)
      when (s1_is_br(w)) {
        printf("hit with br")
      }
      .elsewhen(s1_is_jal(w)){
        printf("hit with jal")
      }
      printf("\nvalid: %d, pc: 0x%x, target: 0x%x\n", s1_resp(w).valid, s1_idx << 3.U, s1_resp(w).bits)
    }

    io.resp.f2(w) := io.resp_in(0).f2(w)
    io.resp.f3(w) := io.resp_in(0).f3(w)
    when (RegNext(s1_hits(w))) {
      io.resp.f2(w).predicted_pc := RegNext(s1_resp(w))
      io.resp.f2(w).is_br        := RegNext(s1_is_br(w))
      io.resp.f2(w).is_jal       := RegNext(s1_is_jal(w))
      io.resp.f2(w).pred_set.bits     := RegNext(entry_pred_set)
      io.resp.f2(w).pred_set.valid := RegNext(entry_pred_set_valid)
      when (RegNext(s1_is_jal(w))) {
        io.resp.f2(w).taken      := true.B
      }
      when (debug_flag) {
        printf("btb f2, cycle: %d", debug_cycles.value)
        when (io.resp.f2(w).is_br) {
          printf("with br")
        }
        .elsewhen(io.resp.f2(w).is_jal){
          printf("with jal")
        }
        printf("\nvalid: %d, pc: 0x%x, target: 0x%x, taken: %d\n", io.resp.f2(w).predicted_pc.valid, RegNext(s1_idx) << 3.U, io.resp.f2(w).predicted_pc.bits, io.resp.f2(w).taken)
      }
    }
    when (RegNext(RegNext(s1_hits(w)))) {
      io.resp.f3(w).predicted_pc := RegNext(io.resp.f2(w).predicted_pc)
      io.resp.f3(w).is_br        := RegNext(io.resp.f2(w).is_br)
      io.resp.f3(w).is_jal       := RegNext(io.resp.f2(w).is_jal)
      io.resp.f3(w).pred_set.bits     := RegNext(io.resp.f2(w).pred_set.bits)
      io.resp.f3(w).pred_set.valid := RegNext(io.resp.f2(w).pred_set.valid)
      
      when (RegNext(RegNext(s1_is_jal(w)))) {
        io.resp.f3(w).taken      := true.B
      }
      when (debug_flag) {
        printf("btb f3, cycle: %d", debug_cycles.value)
        when (io.resp.f3(w).is_br) {
          printf("with br")
        }
        .elsewhen(io.resp.f3(w).is_jal){
          printf("with jal")
        }
        printf("\nvalid: %d, pc: 0x%x, target: 0x%x, taken: %d\n", io.resp.f3(w).predicted_pc.valid, RegNext(RegNext(s1_idx)) << 3.U, io.resp.f3(w).predicted_pc.bits, io.resp.f3(w).taken)
      }
    }
  }

  val alloc_way = if (nWays > 1) {
    val r_metas = Cat(VecInit(s1_req_rmeta.map { w => VecInit(w.map(_.tag)) }).asUInt, s1_req_tag(tagSz-1,0))
    val l = log2Ceil(nWays)
    val nChunks = (r_metas.getWidth + l - 1) / l
    val chunks = (0 until nChunks) map { i =>
      r_metas(min((i+1)*l, r_metas.getWidth)-1, i*l)
    }
    chunks.reduce(_^_)
  } else {
    0.U
  }
  s1_meta.write_way := Mux(s1_hits.reduce(_||_),
    PriorityEncoder(s1_hit_ohs.map(_.asUInt).reduce(_|_)),
    alloc_way)

  // val s1_update = RegNext(io.update)
  // val s1_update_idx = fetchIdx(s1_update.pc)
  val s1_update_cfi_idx = s1_update.bits.cfi_idx.bits
  // class BTBPredictMeta extends Bundle {val write_way = UInt(log2Ceil(nWays).W)}
  val s1_update_meta    = s1_update.bits.meta.asTypeOf(new BTBPredictMeta)

  val max_offset_value = Cat(0.B, ~(0.U((offsetSz-1).W))).asSInt
  val min_offset_value = Cat(1.B,  (0.U((offsetSz-1).W))).asSInt
  val new_offset_value = (s1_update.bits.target.asSInt -
    (s1_update.bits.pc + (s1_update.bits.cfi_idx.bits << 1)).asSInt)
  val offset_is_extended = (new_offset_value > max_offset_value ||
                            new_offset_value < min_offset_value)


  val s1_update_wbtb_data  = Wire(new BTBEntry)
  s1_update_wbtb_data.extended := offset_is_extended
  s1_update_wbtb_data.offset   := new_offset_value
  val s1_update_wbtb_mask = (UIntToOH(s1_update_cfi_idx) &
    Fill(bankWidth, s1_update.bits.cfi_idx.valid && s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.is_commit_update))

  val s1_update_wmeta_mask = ((s1_update_wbtb_mask | s1_update.bits.br_mask) &
    (Fill(bankWidth, s1_update.valid && s1_update.bits.is_commit_update) |
     (Fill(bankWidth, s1_update.valid) & s1_update.bits.btb_mispredicts)
    )
  )
  val s1_update_wmeta_data = Wire(Vec(bankWidth, new BTBMeta))

  val s1_update_wpred_set_data = Wire(new BTBPredSetIndex)
  s1_update_wpred_set_data.pred_set := io.pred_set_update.bits.pred_set
  s1_update_wpred_set_data.valid := true.B
  val s1_update_wpred_set_mask = UIntToOH(io.pred_set_update.bits.cfi_idx)
  val update_pc = fetchIdx(io.pred_set_update.bits.pc)

  for (w <- 0 until bankWidth) {
    s1_update_wmeta_data(w).tag     := Mux(s1_update.bits.btb_mispredicts(w), 0.U, s1_update_idx >> log2Ceil(nSets))
    s1_update_wmeta_data(w).is_br   := s1_update.bits.br_mask(w)
  }

  for (w <- 0 until nWays) {
    when (doing_reset || s1_update_meta.write_way === w.U || (w == 0 && nWays == 1).B) {
      btb(w).write(
        Mux(doing_reset,
          reset_idx,
          s1_update_idx),
        Mux(doing_reset,
          VecInit(Seq.fill(bankWidth) { 0.U(btbEntrySz.W) }),
          VecInit(Seq.fill(bankWidth) { s1_update_wbtb_data.asUInt })),
        Mux(doing_reset,
          (~(0.U(bankWidth.W))),
          s1_update_wbtb_mask).asBools
      )
      meta(w).write(
        Mux(doing_reset,
          reset_idx,
          s1_update_idx),
        Mux(doing_reset,
          VecInit(Seq.fill(bankWidth) { 0.U(btbMetaSz.W) }),
          VecInit(s1_update_wmeta_data.map(_.asUInt))),
        Mux(doing_reset,
          (~(0.U(bankWidth.W))),
          s1_update_wmeta_mask).asBools
      )
      // When update BTB, flush pred_set by set valid to false
      pset(w).write(
        Mux(doing_reset, 
          reset_idx,
          s1_update_idx),
        VecInit(Seq.fill(bankWidth) { 0.U(btbpredSetIndexSz.W) }),
        Mux(doing_reset,
          (~(0.U(bankWidth.W))),
          s1_update_wmeta_mask).asBools
      )

    }
  }
  when (s1_update_wbtb_mask =/= 0.U && offset_is_extended) {
    ebtb.write(s1_update_idx, s1_update.bits.target)
  }


  when (s1_hits.reduce(_||_)) {
    s1_btb_hit_index.way := s1_meta.write_way
    s1_btb_hit_index.valid := true.B
    s1_btb_hit_index.pc := s1_idx

    when (debug_flag) {
      printf("btb hit s1, cycle: %d", debug_cycles.value)
      printf("\npc: 0x%x, way: %d\n", s1_idx << 3.U, s1_meta.write_way)
    }
  }

  // TODO: will it hit 3 times in one visit?
  for (w <- 0 until nWays) {
    when ((s1_btb_hit_index.way === w.U || (w == 0 && nWays == 1).B) &&
      io.pred_set_update.valid && s1_btb_hit_index.valid && (update_pc === s1_btb_hit_index.pc)) {
        pset(w).write(
          s1_btb_hit_index.pc,
          VecInit(Seq.fill(bankWidth) {s1_update_wpred_set_data.asUInt}),
          s1_update_wpred_set_mask.asBools
        )
        when (debug_flag) {
          printf("btb update s1, cycle: %d", debug_cycles.value)
          printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s1_btb_hit_index.way)
        }
    }
    .elsewhen ((s2_btb_hit_index.way === w.U || (w == 0 && nWays == 1).B) &&
      io.pred_set_update.valid && s2_btb_hit_index.valid && (update_pc === s2_btb_hit_index.pc)) {
        pset(w).write(
          s2_btb_hit_index.pc,
          VecInit(Seq.fill(bankWidth) {s1_update_wpred_set_data.asUInt}),
          s1_update_wpred_set_mask.asBools
        )
        when (debug_flag) {
          printf("btb update s2, cycle: %d", debug_cycles.value)
          printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s2_btb_hit_index.way)
        }
    }
    .elsewhen ((s3_btb_hit_index.way === w.U || (w == 0 && nWays == 1).B) &&
      io.pred_set_update.valid && s3_btb_hit_index.valid && (update_pc === s3_btb_hit_index.pc)) {
        pset(w).write(
          s3_btb_hit_index.pc,
          VecInit(Seq.fill(bankWidth) {s1_update_wpred_set_data.asUInt}),
          s1_update_wpred_set_mask.asBools
        )
        when (debug_flag) {
          printf("btb update s1, cycle: %d", debug_cycles.value)
          printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s3_btb_hit_index.way)
        }
    } 
  }
}

