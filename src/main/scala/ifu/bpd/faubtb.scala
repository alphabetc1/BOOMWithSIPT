package boom.ifu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.common._
import boom.util.{BoomCoreStringPrefix, WrapInc}

import scala.math.min

case class BoomFAMicroBTBParams(
  nWays: Int = 16,
  offsetSz: Int = 13
)


class FAMicroBTBBranchPredictorBank(params: BoomFAMicroBTBParams = BoomFAMicroBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  override val nWays         = params.nWays
  val tagSz         = vaddrBitsExtended - log2Ceil(fetchWidth) - 1
  val offsetSz      = params.offsetSz
  val nWrBypassEntries = 2

  def bimWrite(v: UInt, taken: Bool): UInt = {
    val old_bim_sat_taken  = v === 3.U
    val old_bim_sat_ntaken = v === 0.U
    Mux(old_bim_sat_taken  &&  taken, 3.U,
      Mux(old_bim_sat_ntaken && !taken, 0.U,
      Mux(taken, v + 1.U, v - 1.U)))
  }

  require(isPow2(nWays))

  class MicroBTBEntry extends Bundle {
    val offset   = SInt(offsetSz.W)
  }

  class MicroBTBMeta extends Bundle {
    val is_br = Bool()
    val tag   = UInt(tagSz.W)
    val ctr   = UInt(2.W)
  }

  class MicroBTBPredictMeta extends Bundle {
    val hits  = Vec(bankWidth, Bool())
    val write_way = UInt(log2Ceil(nWays).W)
  }

  // Update MicroBTB entry when branch jump completes
  class MicroBTBIndexInfo extends Bundle{
    val pc = UInt(vaddrBitsExtended.W)
    val way = UInt(log2Ceil(nWays).W)
    val valid = Bool()
  }

  // Info about pred set index
  class MicroBTBPredSetIndex extends Bundle{
    val pred_set = UInt(predSetBits.W)
    val valid = Bool()
  }
  val btbpredSetIndexSz = 3

  val s1_meta = Wire(new MicroBTBPredictMeta)
  override val metaSz = s1_meta.asUInt.getWidth


  val meta     = RegInit((0.U).asTypeOf(Vec(nWays, Vec(bankWidth, new MicroBTBMeta))))
  val btb      = Reg(Vec(nWays, Vec(bankWidth, new MicroBTBEntry)))

  // Info about pred_set_index
  val pset = RegInit((0.U).asTypeOf(Vec(nWays, Vec(bankWidth, new MicroBTBPredSetIndex))))
  val s1_btb_hit_index = RegInit((0.U).asTypeOf(new MicroBTBIndexInfo()))
  val s2_btb_hit_index = RegNext(s1_btb_hit_index)
  val s3_btb_hit_index = RegNext(s2_btb_hit_index)

  val mems = Nil

  val s1_req_tag   = s1_idx


  val s1_resp   = Wire(Vec(bankWidth, Valid(UInt(vaddrBitsExtended.W))))
  val s1_taken  = Wire(Vec(bankWidth, Bool()))
  val s1_is_br  = Wire(Vec(bankWidth, Bool()))
  val s1_is_jal = Wire(Vec(bankWidth, Bool()))
  val s1_pred_set = Wire(Vec(bankWidth, Valid(UInt(predSetBits.W))))

  val s1_hit_ohs = VecInit((0 until bankWidth) map { i =>
    VecInit((0 until nWays) map { w =>
      meta(w)(i).tag === s1_req_tag(tagSz-1,0)
    })
  })
  val s1_hits     = s1_hit_ohs.map { oh => oh.reduce(_||_) }
  val s1_hit_ways = s1_hit_ohs.map { oh => PriorityEncoder(oh) }

  // Debug info
  val debug_flag = false.B
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)

  for (w <- 0 until bankWidth) {
    val entry_meta = meta(s1_hit_ways(w))(w)
    val entry_pred_set = pset(s1_hit_ways(w))(w)
    s1_resp(w).valid := s1_valid && s1_hits(w)
    s1_resp(w).bits  := (s1_pc.asSInt + (w << 1).S + btb(s1_hit_ways(w))(w).offset).asUInt
    s1_is_br(w)      := s1_resp(w).valid &&  entry_meta.is_br
    s1_is_jal(w)     := s1_resp(w).valid && !entry_meta.is_br
    s1_taken(w)      := !entry_meta.is_br || entry_meta.ctr(1)
    s1_pred_set(w).bits := entry_pred_set.pred_set
    s1_pred_set(w).valid := entry_pred_set.valid

    s1_meta.hits(w)     := s1_hits(w)
  }
  val alloc_way = {
    val r_metas = Cat(VecInit(meta.map(e => VecInit(e.map(_.tag)))).asUInt, s1_idx(tagSz-1,0))
    val l = log2Ceil(nWays)
    val nChunks = (r_metas.getWidth + l - 1) / l
    val chunks = (0 until nChunks) map { i =>
      r_metas(min((i+1)*l, r_metas.getWidth)-1, i*l)
    }
    chunks.reduce(_^_)
  }
  s1_meta.write_way := Mux(s1_hits.reduce(_||_),
    PriorityEncoder(s1_hit_ohs.map(_.asUInt).reduce(_|_)),
    alloc_way)

  for (w <- 0 until bankWidth) {
    io.resp.f1(w).predicted_pc   := s1_resp(w)
    io.resp.f1(w).is_br          := s1_is_br(w)
    io.resp.f1(w).is_jal         := s1_is_jal(w)
    io.resp.f1(w).taken          := s1_taken(w)
    io.resp.f1(w).pred_set.bits       := s1_pred_set(w).bits
    io.resp.f1(w).pred_set.valid := s1_pred_set(w).valid

    when (debug_flag && s1_hits(w)) {
      printf("microBtb f1, cycle: %d", debug_cycles.value)
      when (s1_is_br(w)) {
        printf("hit with br")
      }
      .elsewhen(s1_is_jal(w)){
        printf("hit with jal")
      }
      printf("\nvalid: %d, pc: 0x%x, target: 0x%x, taken: %d\n", s1_resp(w).valid, s1_idx << 3.U, s1_resp(w).bits, s1_taken(w))
    }

    io.resp.f2(w) := RegNext(io.resp.f1(w))
    io.resp.f3(w) := RegNext(io.resp.f2(w))
  }
  io.f3_meta := RegNext(RegNext(s1_meta.asUInt))

  val s1_update_cfi_idx = s1_update.bits.cfi_idx.bits
  val s1_update_meta    = s1_update.bits.meta.asTypeOf(new MicroBTBPredictMeta)
  val s1_update_write_way = s1_update_meta.write_way

  val max_offset_value = (~(0.U)((offsetSz-1).W)).asSInt
  val min_offset_value = Cat(1.B, (0.U)((offsetSz-1).W)).asSInt
  val new_offset_value = (s1_update.bits.target.asSInt -
    (s1_update.bits.pc + (s1_update.bits.cfi_idx.bits << 1)).asSInt)

  val s1_update_wbtb_data     = Wire(new MicroBTBEntry)
  s1_update_wbtb_data.offset := new_offset_value
  val s1_update_wbtb_mask = (UIntToOH(s1_update_cfi_idx) &
    Fill(bankWidth, s1_update.bits.cfi_idx.valid && s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.is_commit_update))

  val s1_update_wmeta_mask = ((s1_update_wbtb_mask | s1_update.bits.br_mask) &
    Fill(bankWidth, s1_update.valid && s1_update.bits.is_commit_update))

  // Write the BTB with the target
  when (s1_update.valid && s1_update.bits.cfi_taken && s1_update.bits.cfi_idx.valid && s1_update.bits.is_commit_update) {
    btb(s1_update_write_way)(s1_update_cfi_idx).offset := new_offset_value
    pset(s1_update_write_way)(s1_update_cfi_idx).valid := 0.U
  }

  // Write the meta
  for (w <- 0 until bankWidth) {
    when (s1_update.valid && s1_update.bits.is_commit_update &&
      (s1_update.bits.br_mask(w) ||
        (s1_update_cfi_idx === w.U && s1_update.bits.cfi_taken && s1_update.bits.cfi_idx.valid))) {
      val was_taken = (s1_update_cfi_idx === w.U && s1_update.bits.cfi_idx.valid &&
        (s1_update.bits.cfi_taken || s1_update.bits.cfi_is_jal))

      meta(s1_update_write_way)(w).is_br := s1_update.bits.br_mask(w)
      meta(s1_update_write_way)(w).tag   := s1_update_idx
      meta(s1_update_write_way)(w).ctr   := Mux(!s1_update_meta.hits(w),
        Mux(was_taken, 3.U, 0.U),
        bimWrite(meta(s1_update_write_way)(w).ctr, was_taken)
      )
    }
  }

  when (s1_hits.reduce(_||_)) {
    s1_btb_hit_index.way := s1_meta.write_way
    s1_btb_hit_index.valid := true.B
    s1_btb_hit_index.pc := s1_idx

    when (debug_flag) {
      printf("microBtb hit s1, cycle: %d", debug_cycles.value)
      printf("\npc: 0x%x, way: %d\n", s1_idx << 3.U, s1_meta.write_way)
    }
  }.otherwise {
    s1_btb_hit_index.valid := false.B
  }

  // Update pred_set in MicroBTB after address translation
  val update_pc = fetchIdx(io.pred_set_update.bits.pc)
  when(io.pred_set_update.valid && (update_pc === s1_btb_hit_index.pc) && s1_btb_hit_index.valid){
    pset(s1_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).valid := true.B
    pset(s1_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).pred_set := io.pred_set_update.bits.pred_set
    when (debug_flag) {
      printf("microBtb update s1, cycle: %d", debug_cycles.value)
      printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s1_btb_hit_index.way)
    }
  }
  .elsewhen(io.pred_set_update.valid && (update_pc === s2_btb_hit_index.pc) && s2_btb_hit_index.valid){
    pset(s2_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).valid := true.B
    pset(s2_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).pred_set := io.pred_set_update.bits.pred_set
    when (debug_flag) {
      printf("microBtb update s2, cycle: %d", debug_cycles.value)
      printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s2_btb_hit_index.way)
    }
  }
  .elsewhen(io.pred_set_update.valid && (update_pc === s3_btb_hit_index.pc) && s3_btb_hit_index.valid){
    pset(s3_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).valid := true.B
    pset(s3_btb_hit_index.way)(io.pred_set_update.bits.cfi_idx).pred_set := io.pred_set_update.bits.pred_set
    when (debug_flag) {
      printf("microBtb update s3, cycle: %d", debug_cycles.value)
      printf("\npc: 0x%x, cif_idx: %d, pred_set: %d, way: %d\n", update_pc << 3.U, io.pred_set_update.bits.cfi_idx, io.pred_set_update.bits.pred_set, s3_btb_hit_index.way)
    }
  }
}

