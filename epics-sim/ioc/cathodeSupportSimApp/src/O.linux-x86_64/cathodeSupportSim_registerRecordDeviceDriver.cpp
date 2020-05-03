/* THIS IS A GENERATED FILE. DO NOT EDIT! */
/* Generated from ../O.Common/cathodeSupportSim.dbd */

#include <string.h>

#include "compilerDependencies.h"
#include "epicsStdlib.h"
#include "iocsh.h"
#include "iocshRegisterCommon.h"
#include "registryCommon.h"

extern "C" {

epicsShareExtern rset *pvar_rset_aSubRSET, *pvar_rset_aaiRSET,
    *pvar_rset_aaoRSET, *pvar_rset_acalcoutRSET, *pvar_rset_aiRSET,
    *pvar_rset_aoRSET, *pvar_rset_asynRSET, *pvar_rset_biRSET,
    *pvar_rset_boRSET, *pvar_rset_busyRSET, *pvar_rset_calcRSET,
    *pvar_rset_calcoutRSET, *pvar_rset_compressRSET,
    *pvar_rset_dfanoutRSET, *pvar_rset_eventRSET, *pvar_rset_fanoutRSET,
    *pvar_rset_histogramRSET, *pvar_rset_longinRSET,
    *pvar_rset_longoutRSET, *pvar_rset_lsiRSET, *pvar_rset_lsoRSET,
    *pvar_rset_mbbiRSET, *pvar_rset_mbbiDirectRSET, *pvar_rset_mbboRSET,
    *pvar_rset_mbboDirectRSET, *pvar_rset_motorRSET,
    *pvar_rset_permissiveRSET, *pvar_rset_printfRSET,
    *pvar_rset_scalcoutRSET, *pvar_rset_selRSET, *pvar_rset_seqRSET,
    *pvar_rset_sseqRSET, *pvar_rset_stateRSET, *pvar_rset_stringinRSET,
    *pvar_rset_stringoutRSET, *pvar_rset_subRSET, *pvar_rset_subArrayRSET,
    *pvar_rset_swaitRSET, *pvar_rset_transformRSET,
    *pvar_rset_waveformRSET;

typedef int (*rso_func)(dbRecordType *pdbRecordType);
epicsShareExtern rso_func pvar_func_aSubRecordSizeOffset,
    pvar_func_aaiRecordSizeOffset, pvar_func_aaoRecordSizeOffset,
    pvar_func_acalcoutRecordSizeOffset, pvar_func_aiRecordSizeOffset,
    pvar_func_aoRecordSizeOffset, pvar_func_asynRecordSizeOffset,
    pvar_func_biRecordSizeOffset, pvar_func_boRecordSizeOffset,
    pvar_func_busyRecordSizeOffset, pvar_func_calcRecordSizeOffset,
    pvar_func_calcoutRecordSizeOffset, pvar_func_compressRecordSizeOffset,
    pvar_func_dfanoutRecordSizeOffset, pvar_func_eventRecordSizeOffset,
    pvar_func_fanoutRecordSizeOffset, pvar_func_histogramRecordSizeOffset,
    pvar_func_longinRecordSizeOffset, pvar_func_longoutRecordSizeOffset,
    pvar_func_lsiRecordSizeOffset, pvar_func_lsoRecordSizeOffset,
    pvar_func_mbbiRecordSizeOffset, pvar_func_mbbiDirectRecordSizeOffset,
    pvar_func_mbboRecordSizeOffset, pvar_func_mbboDirectRecordSizeOffset,
    pvar_func_motorRecordSizeOffset, pvar_func_permissiveRecordSizeOffset,
    pvar_func_printfRecordSizeOffset, pvar_func_scalcoutRecordSizeOffset,
    pvar_func_selRecordSizeOffset, pvar_func_seqRecordSizeOffset,
    pvar_func_sseqRecordSizeOffset, pvar_func_stateRecordSizeOffset,
    pvar_func_stringinRecordSizeOffset,
    pvar_func_stringoutRecordSizeOffset, pvar_func_subRecordSizeOffset,
    pvar_func_subArrayRecordSizeOffset, pvar_func_swaitRecordSizeOffset,
    pvar_func_transformRecordSizeOffset,
    pvar_func_waveformRecordSizeOffset;

static const char * const recordTypeNames[] = {
    "aSub", "aai", "aao", "acalcout", "ai", "ao", "asyn", "bi", "bo",
    "busy", "calc", "calcout", "compress", "dfanout", "event", "fanout",
    "histogram", "longin", "longout", "lsi", "lso", "mbbi", "mbbiDirect",
    "mbbo", "mbboDirect", "motor", "permissive", "printf", "scalcout",
    "sel", "seq", "sseq", "state", "stringin", "stringout", "sub",
    "subArray", "swait", "transform", "waveform"
};

static const recordTypeLocation rtl[] = {
    {pvar_rset_aSubRSET, pvar_func_aSubRecordSizeOffset},
    {pvar_rset_aaiRSET, pvar_func_aaiRecordSizeOffset},
    {pvar_rset_aaoRSET, pvar_func_aaoRecordSizeOffset},
    {pvar_rset_acalcoutRSET, pvar_func_acalcoutRecordSizeOffset},
    {pvar_rset_aiRSET, pvar_func_aiRecordSizeOffset},
    {pvar_rset_aoRSET, pvar_func_aoRecordSizeOffset},
    {pvar_rset_asynRSET, pvar_func_asynRecordSizeOffset},
    {pvar_rset_biRSET, pvar_func_biRecordSizeOffset},
    {pvar_rset_boRSET, pvar_func_boRecordSizeOffset},
    {pvar_rset_busyRSET, pvar_func_busyRecordSizeOffset},
    {pvar_rset_calcRSET, pvar_func_calcRecordSizeOffset},
    {pvar_rset_calcoutRSET, pvar_func_calcoutRecordSizeOffset},
    {pvar_rset_compressRSET, pvar_func_compressRecordSizeOffset},
    {pvar_rset_dfanoutRSET, pvar_func_dfanoutRecordSizeOffset},
    {pvar_rset_eventRSET, pvar_func_eventRecordSizeOffset},
    {pvar_rset_fanoutRSET, pvar_func_fanoutRecordSizeOffset},
    {pvar_rset_histogramRSET, pvar_func_histogramRecordSizeOffset},
    {pvar_rset_longinRSET, pvar_func_longinRecordSizeOffset},
    {pvar_rset_longoutRSET, pvar_func_longoutRecordSizeOffset},
    {pvar_rset_lsiRSET, pvar_func_lsiRecordSizeOffset},
    {pvar_rset_lsoRSET, pvar_func_lsoRecordSizeOffset},
    {pvar_rset_mbbiRSET, pvar_func_mbbiRecordSizeOffset},
    {pvar_rset_mbbiDirectRSET, pvar_func_mbbiDirectRecordSizeOffset},
    {pvar_rset_mbboRSET, pvar_func_mbboRecordSizeOffset},
    {pvar_rset_mbboDirectRSET, pvar_func_mbboDirectRecordSizeOffset},
    {pvar_rset_motorRSET, pvar_func_motorRecordSizeOffset},
    {pvar_rset_permissiveRSET, pvar_func_permissiveRecordSizeOffset},
    {pvar_rset_printfRSET, pvar_func_printfRecordSizeOffset},
    {pvar_rset_scalcoutRSET, pvar_func_scalcoutRecordSizeOffset},
    {pvar_rset_selRSET, pvar_func_selRecordSizeOffset},
    {pvar_rset_seqRSET, pvar_func_seqRecordSizeOffset},
    {pvar_rset_sseqRSET, pvar_func_sseqRecordSizeOffset},
    {pvar_rset_stateRSET, pvar_func_stateRecordSizeOffset},
    {pvar_rset_stringinRSET, pvar_func_stringinRecordSizeOffset},
    {pvar_rset_stringoutRSET, pvar_func_stringoutRecordSizeOffset},
    {pvar_rset_subRSET, pvar_func_subRecordSizeOffset},
    {pvar_rset_subArrayRSET, pvar_func_subArrayRecordSizeOffset},
    {pvar_rset_swaitRSET, pvar_func_swaitRecordSizeOffset},
    {pvar_rset_transformRSET, pvar_func_transformRecordSizeOffset},
    {pvar_rset_waveformRSET, pvar_func_waveformRecordSizeOffset}
};

epicsShareExtern dset *pvar_dset_devAaiSoft, *pvar_dset_devAaoSoft,
    *pvar_dset_devaCalcoutSoft, *pvar_dset_devAiSoft,
    *pvar_dset_devAiSoftRaw, *pvar_dset_devAiSoftCallback,
    *pvar_dset_devTimestampAI, *pvar_dset_devAiGeneralTime,
    *pvar_dset_asynAiInt32, *pvar_dset_asynAiInt32Average,
    *pvar_dset_asynAiFloat64, *pvar_dset_asynAiFloat64Average,
    *pvar_dset_asynAiInt64, *pvar_dset_devAiStats, *pvar_dset_devAiClusts,
    *pvar_dset_devAoSoft, *pvar_dset_devAoSoftRaw,
    *pvar_dset_devAoSoftCallback, *pvar_dset_asynAoInt32,
    *pvar_dset_asynAoFloat64, *pvar_dset_asynAoInt64,
    *pvar_dset_devAoStats, *pvar_dset_asynRecordDevice,
    *pvar_dset_devBiSoft, *pvar_dset_devBiSoftRaw,
    *pvar_dset_devBiSoftCallback, *pvar_dset_devBiDbState,
    *pvar_dset_asynBiInt32, *pvar_dset_asynBiUInt32Digital,
    *pvar_dset_devBoSoft, *pvar_dset_devBoSoftRaw,
    *pvar_dset_devBoSoftCallback, *pvar_dset_devBoGeneralTime,
    *pvar_dset_devBoDbState, *pvar_dset_asynBoInt32,
    *pvar_dset_asynBoUInt32Digital, *pvar_dset_devBusySoft,
    *pvar_dset_devBusySoftRaw, *pvar_dset_asynBusyInt32,
    *pvar_dset_devCalcoutSoft, *pvar_dset_devCalcoutSoftCallback,
    *pvar_dset_devEventSoft, *pvar_dset_devHistogramSoft,
    *pvar_dset_devLiSoft, *pvar_dset_devLiSoftCallback,
    *pvar_dset_devLiGeneralTime, *pvar_dset_asynLiInt32,
    *pvar_dset_asynLiUInt32Digital, *pvar_dset_asynLiInt64,
    *pvar_dset_devLoSoft, *pvar_dset_devLoSoftCallback,
    *pvar_dset_asynLoInt32, *pvar_dset_asynLoUInt32Digital,
    *pvar_dset_asynLoInt64, *pvar_dset_devLsiSoft,
    *pvar_dset_devLsiEnviron, *pvar_dset_devLsoSoft,
    *pvar_dset_devLsoSoftCallback, *pvar_dset_devLsoStdio,
    *pvar_dset_devMbbiSoft, *pvar_dset_devMbbiSoftRaw,
    *pvar_dset_devMbbiSoftCallback, *pvar_dset_asynMbbiInt32,
    *pvar_dset_asynMbbiUInt32Digital, *pvar_dset_devMbbiDirectSoft,
    *pvar_dset_devMbbiDirectSoftRaw, *pvar_dset_devMbbiDirectSoftCallback,
    *pvar_dset_asynMbbiDirectUInt32Digital, *pvar_dset_devMbboSoft,
    *pvar_dset_devMbboSoftRaw, *pvar_dset_devMbboSoftCallback,
    *pvar_dset_asynMbboInt32, *pvar_dset_asynMbboUInt32Digital,
    *pvar_dset_devMbboDirectSoft, *pvar_dset_devMbboDirectSoftRaw,
    *pvar_dset_devMbboDirectSoftCallback,
    *pvar_dset_asynMbboDirectUInt32Digital, *pvar_dset_devMotorSoft,
    *pvar_dset_devMotorAsyn, *pvar_dset_devMotorSim,
    *pvar_dset_devPrintfSoft, *pvar_dset_devPrintfSoftCallback,
    *pvar_dset_devPrintfStdio, *pvar_dset_devsCalcoutSoft,
    *pvar_dset_devSiSoft, *pvar_dset_devSiSoftCallback,
    *pvar_dset_devTimestampSI, *pvar_dset_devSiGeneralTime,
    *pvar_dset_devSiEnviron, *pvar_dset_asynSiOctetCmdResponse,
    *pvar_dset_asynSiOctetWriteRead, *pvar_dset_asynSiOctetRead,
    *pvar_dset_devStringinStats, *pvar_dset_devStringinEnvVar,
    *pvar_dset_devStringinEpics, *pvar_dset_devSoSoft,
    *pvar_dset_devSoSoftCallback, *pvar_dset_devSoStdio,
    *pvar_dset_asynSoOctetWrite, *pvar_dset_devSASoft,
    *pvar_dset_devSWaitIoEvent, *pvar_dset_devWfSoft,
    *pvar_dset_asynWfOctetCmdResponse, *pvar_dset_asynWfOctetWriteRead,
    *pvar_dset_asynWfOctetRead, *pvar_dset_asynWfOctetWrite,
    *pvar_dset_asynWfOctetWriteBinary, *pvar_dset_asynInt8ArrayWfIn,
    *pvar_dset_asynInt8ArrayWfOut, *pvar_dset_asynInt16ArrayWfIn,
    *pvar_dset_asynInt16ArrayWfOut, *pvar_dset_asynInt32ArrayWfIn,
    *pvar_dset_asynInt32ArrayWfOut, *pvar_dset_asynInt32TimeSeries,
    *pvar_dset_asynFloat32ArrayWfIn, *pvar_dset_asynFloat32ArrayWfOut,
    *pvar_dset_asynFloat64ArrayWfIn, *pvar_dset_asynFloat64ArrayWfOut,
    *pvar_dset_asynFloat64TimeSeries, *pvar_dset_devWaveformStats;

static const char * const deviceSupportNames[] = {
    "devAaiSoft", "devAaoSoft", "devaCalcoutSoft", "devAiSoft",
    "devAiSoftRaw", "devAiSoftCallback", "devTimestampAI",
    "devAiGeneralTime", "asynAiInt32", "asynAiInt32Average",
    "asynAiFloat64", "asynAiFloat64Average", "asynAiInt64", "devAiStats",
    "devAiClusts", "devAoSoft", "devAoSoftRaw", "devAoSoftCallback",
    "asynAoInt32", "asynAoFloat64", "asynAoInt64", "devAoStats",
    "asynRecordDevice", "devBiSoft", "devBiSoftRaw", "devBiSoftCallback",
    "devBiDbState", "asynBiInt32", "asynBiUInt32Digital", "devBoSoft",
    "devBoSoftRaw", "devBoSoftCallback", "devBoGeneralTime",
    "devBoDbState", "asynBoInt32", "asynBoUInt32Digital", "devBusySoft",
    "devBusySoftRaw", "asynBusyInt32", "devCalcoutSoft",
    "devCalcoutSoftCallback", "devEventSoft", "devHistogramSoft",
    "devLiSoft", "devLiSoftCallback", "devLiGeneralTime", "asynLiInt32",
    "asynLiUInt32Digital", "asynLiInt64", "devLoSoft",
    "devLoSoftCallback", "asynLoInt32", "asynLoUInt32Digital",
    "asynLoInt64", "devLsiSoft", "devLsiEnviron", "devLsoSoft",
    "devLsoSoftCallback", "devLsoStdio", "devMbbiSoft", "devMbbiSoftRaw",
    "devMbbiSoftCallback", "asynMbbiInt32", "asynMbbiUInt32Digital",
    "devMbbiDirectSoft", "devMbbiDirectSoftRaw",
    "devMbbiDirectSoftCallback", "asynMbbiDirectUInt32Digital",
    "devMbboSoft", "devMbboSoftRaw", "devMbboSoftCallback",
    "asynMbboInt32", "asynMbboUInt32Digital", "devMbboDirectSoft",
    "devMbboDirectSoftRaw", "devMbboDirectSoftCallback",
    "asynMbboDirectUInt32Digital", "devMotorSoft", "devMotorAsyn",
    "devMotorSim", "devPrintfSoft", "devPrintfSoftCallback",
    "devPrintfStdio", "devsCalcoutSoft", "devSiSoft", "devSiSoftCallback",
    "devTimestampSI", "devSiGeneralTime", "devSiEnviron",
    "asynSiOctetCmdResponse", "asynSiOctetWriteRead", "asynSiOctetRead",
    "devStringinStats", "devStringinEnvVar", "devStringinEpics",
    "devSoSoft", "devSoSoftCallback", "devSoStdio", "asynSoOctetWrite",
    "devSASoft", "devSWaitIoEvent", "devWfSoft", "asynWfOctetCmdResponse",
    "asynWfOctetWriteRead", "asynWfOctetRead", "asynWfOctetWrite",
    "asynWfOctetWriteBinary", "asynInt8ArrayWfIn", "asynInt8ArrayWfOut",
    "asynInt16ArrayWfIn", "asynInt16ArrayWfOut", "asynInt32ArrayWfIn",
    "asynInt32ArrayWfOut", "asynInt32TimeSeries", "asynFloat32ArrayWfIn",
    "asynFloat32ArrayWfOut", "asynFloat64ArrayWfIn",
    "asynFloat64ArrayWfOut", "asynFloat64TimeSeries", "devWaveformStats"
};

static const dset * const devsl[] = {
    pvar_dset_devAaiSoft, pvar_dset_devAaoSoft, pvar_dset_devaCalcoutSoft,
    pvar_dset_devAiSoft, pvar_dset_devAiSoftRaw,
    pvar_dset_devAiSoftCallback, pvar_dset_devTimestampAI,
    pvar_dset_devAiGeneralTime, pvar_dset_asynAiInt32,
    pvar_dset_asynAiInt32Average, pvar_dset_asynAiFloat64,
    pvar_dset_asynAiFloat64Average, pvar_dset_asynAiInt64,
    pvar_dset_devAiStats, pvar_dset_devAiClusts, pvar_dset_devAoSoft,
    pvar_dset_devAoSoftRaw, pvar_dset_devAoSoftCallback,
    pvar_dset_asynAoInt32, pvar_dset_asynAoFloat64, pvar_dset_asynAoInt64,
    pvar_dset_devAoStats, pvar_dset_asynRecordDevice, pvar_dset_devBiSoft,
    pvar_dset_devBiSoftRaw, pvar_dset_devBiSoftCallback,
    pvar_dset_devBiDbState, pvar_dset_asynBiInt32,
    pvar_dset_asynBiUInt32Digital, pvar_dset_devBoSoft,
    pvar_dset_devBoSoftRaw, pvar_dset_devBoSoftCallback,
    pvar_dset_devBoGeneralTime, pvar_dset_devBoDbState,
    pvar_dset_asynBoInt32, pvar_dset_asynBoUInt32Digital,
    pvar_dset_devBusySoft, pvar_dset_devBusySoftRaw,
    pvar_dset_asynBusyInt32, pvar_dset_devCalcoutSoft,
    pvar_dset_devCalcoutSoftCallback, pvar_dset_devEventSoft,
    pvar_dset_devHistogramSoft, pvar_dset_devLiSoft,
    pvar_dset_devLiSoftCallback, pvar_dset_devLiGeneralTime,
    pvar_dset_asynLiInt32, pvar_dset_asynLiUInt32Digital,
    pvar_dset_asynLiInt64, pvar_dset_devLoSoft,
    pvar_dset_devLoSoftCallback, pvar_dset_asynLoInt32,
    pvar_dset_asynLoUInt32Digital, pvar_dset_asynLoInt64,
    pvar_dset_devLsiSoft, pvar_dset_devLsiEnviron, pvar_dset_devLsoSoft,
    pvar_dset_devLsoSoftCallback, pvar_dset_devLsoStdio,
    pvar_dset_devMbbiSoft, pvar_dset_devMbbiSoftRaw,
    pvar_dset_devMbbiSoftCallback, pvar_dset_asynMbbiInt32,
    pvar_dset_asynMbbiUInt32Digital, pvar_dset_devMbbiDirectSoft,
    pvar_dset_devMbbiDirectSoftRaw, pvar_dset_devMbbiDirectSoftCallback,
    pvar_dset_asynMbbiDirectUInt32Digital, pvar_dset_devMbboSoft,
    pvar_dset_devMbboSoftRaw, pvar_dset_devMbboSoftCallback,
    pvar_dset_asynMbboInt32, pvar_dset_asynMbboUInt32Digital,
    pvar_dset_devMbboDirectSoft, pvar_dset_devMbboDirectSoftRaw,
    pvar_dset_devMbboDirectSoftCallback,
    pvar_dset_asynMbboDirectUInt32Digital, pvar_dset_devMotorSoft,
    pvar_dset_devMotorAsyn, pvar_dset_devMotorSim,
    pvar_dset_devPrintfSoft, pvar_dset_devPrintfSoftCallback,
    pvar_dset_devPrintfStdio, pvar_dset_devsCalcoutSoft,
    pvar_dset_devSiSoft, pvar_dset_devSiSoftCallback,
    pvar_dset_devTimestampSI, pvar_dset_devSiGeneralTime,
    pvar_dset_devSiEnviron, pvar_dset_asynSiOctetCmdResponse,
    pvar_dset_asynSiOctetWriteRead, pvar_dset_asynSiOctetRead,
    pvar_dset_devStringinStats, pvar_dset_devStringinEnvVar,
    pvar_dset_devStringinEpics, pvar_dset_devSoSoft,
    pvar_dset_devSoSoftCallback, pvar_dset_devSoStdio,
    pvar_dset_asynSoOctetWrite, pvar_dset_devSASoft,
    pvar_dset_devSWaitIoEvent, pvar_dset_devWfSoft,
    pvar_dset_asynWfOctetCmdResponse, pvar_dset_asynWfOctetWriteRead,
    pvar_dset_asynWfOctetRead, pvar_dset_asynWfOctetWrite,
    pvar_dset_asynWfOctetWriteBinary, pvar_dset_asynInt8ArrayWfIn,
    pvar_dset_asynInt8ArrayWfOut, pvar_dset_asynInt16ArrayWfIn,
    pvar_dset_asynInt16ArrayWfOut, pvar_dset_asynInt32ArrayWfIn,
    pvar_dset_asynInt32ArrayWfOut, pvar_dset_asynInt32TimeSeries,
    pvar_dset_asynFloat32ArrayWfIn, pvar_dset_asynFloat32ArrayWfOut,
    pvar_dset_asynFloat64ArrayWfIn, pvar_dset_asynFloat64ArrayWfOut,
    pvar_dset_asynFloat64TimeSeries, pvar_dset_devWaveformStats
};

epicsShareExtern drvet *pvar_drvet_drvAsyn, *pvar_drvet_motorSim;

static const char *driverSupportNames[] = {
    "drvAsyn", "motorSim"};

static struct drvet *drvsl[] = {
    pvar_drvet_drvAsyn,
    pvar_drvet_motorSim};

typedef void (*reg_func)(void);
epicsShareExtern reg_func pvar_func_NDAttrPlotRegister,
    pvar_func_NDAttrRegister, pvar_func_NDCircularBuffRegister,
    pvar_func_NDCodecRegister, pvar_func_NDColorConvertRegister,
    pvar_func_NDFFTRegister, pvar_func_NDGatherRegister,
    pvar_func_NDOverlayRegister, pvar_func_NDPosPluginRegister,
    pvar_func_NDProcessRegister, pvar_func_NDROIRegister,
    pvar_func_NDROIStatRegister, pvar_func_NDScatterRegister,
    pvar_func_NDStatsRegister, pvar_func_NDStdArraysRegister,
    pvar_func_NDTimeSeriesRegister, pvar_func_NDTransformRegister,
    pvar_func_arrInitialize, pvar_func_arrayTestRegister, pvar_func_asSub,
    pvar_func_asynInterposeDelayRegister,
    pvar_func_asynInterposeEchoRegister,
    pvar_func_asynInterposeEosRegister,
    pvar_func_asynInterposeFlushRegister,
    pvar_func_asynMotorControllerRegister, pvar_func_asynRegister,
    pvar_func_dbndInitialize, pvar_func_decInitialize,
    pvar_func_editSseqRegistrar, pvar_func_interpRegister,
    pvar_func_motorRegister, pvar_func_motorSimDriverRegister,
    pvar_func_motorSimRegister, pvar_func_motorUtilRegister,
    pvar_func_parseRegister, pvar_func_prosilicaRegister,
    pvar_func_simDetectorRegister, pvar_func_subAveRegister,
    pvar_func_syncInitialize, pvar_func_tsInitialize,
    pvar_func_register_func_myAttrFunct1,
    pvar_func_register_func_myTimeStampSource,
    pvar_func_register_func_rebootProc, pvar_func_register_func_scanMon,
    pvar_func_register_func_scanMonInit;

epicsShareExtern int * const pvar_int_CASDEBUG;
epicsShareExtern int * const pvar_int_aCalcAsyncThreshold;
epicsShareExtern int * const pvar_int_aCalcLoopMax;
epicsShareExtern int * const pvar_int_aCalcPerformDebug;
epicsShareExtern int * const pvar_int_aCalcPostfixDebug;
epicsShareExtern int * const pvar_int_aCalcoutRecordDebug;
epicsShareExtern int * const pvar_int_arrayTestDebug;
epicsShareExtern int * const pvar_int_asCaDebug;
epicsShareExtern int * const pvar_int_atExitDebug;
epicsShareExtern double * const pvar_double_boHIGHlimit;
epicsShareExtern int * const pvar_int_boHIGHprecision;
epicsShareExtern double * const pvar_double_calcoutODLYlimit;
epicsShareExtern int * const pvar_int_calcoutODLYprecision;
epicsShareExtern int * const pvar_int_callbackParallelThreadsDefault;
epicsShareExtern int * const pvar_int_dbBptNotMonotonic;
epicsShareExtern int * const pvar_int_dbQuietMacroWarnings;
epicsShareExtern int * const pvar_int_dbRecordsAbcSorted;
epicsShareExtern int * const pvar_int_dbRecordsOnceOnly;
epicsShareExtern int * const pvar_int_dbTemplateMaxVars;
epicsShareExtern int * const pvar_int_dbThreadRealtimeLock;
epicsShareExtern int * const pvar_int_debugSubAve;
epicsShareExtern int * const pvar_int_devaCalcoutSoftDebug;
epicsShareExtern int * const pvar_int_devsCalcoutSoftDebug;
epicsShareExtern int * const pvar_int_eraseNDAttributes;
epicsShareExtern int * const pvar_int_histogramSDELprecision;
epicsShareExtern int * const pvar_int_interpDebug;
epicsShareExtern int * const pvar_int_sCalcLoopMax;
epicsShareExtern int * const pvar_int_sCalcPerformDebug;
epicsShareExtern int * const pvar_int_sCalcPostfixDebug;
epicsShareExtern int * const pvar_int_sCalcStackHW;
epicsShareExtern int * const pvar_int_sCalcStackLW;
epicsShareExtern int * const pvar_int_sCalcoutRecordDebug;
epicsShareExtern double * const pvar_double_seqDLYlimit;
epicsShareExtern int * const pvar_int_seqDLYprecision;
epicsShareExtern int * const pvar_int_sseqRecDebug;
epicsShareExtern int * const pvar_int_swaitRecordDebug;
epicsShareExtern int * const pvar_int_transformRecordDebug;

static struct iocshVarDef vardefs[] = {
    {"CASDEBUG", iocshArgInt, pvar_int_CASDEBUG},
    {"aCalcAsyncThreshold", iocshArgInt, pvar_int_aCalcAsyncThreshold},
    {"aCalcLoopMax", iocshArgInt, pvar_int_aCalcLoopMax},
    {"aCalcPerformDebug", iocshArgInt, pvar_int_aCalcPerformDebug},
    {"aCalcPostfixDebug", iocshArgInt, pvar_int_aCalcPostfixDebug},
    {"aCalcoutRecordDebug", iocshArgInt, pvar_int_aCalcoutRecordDebug},
    {"arrayTestDebug", iocshArgInt, pvar_int_arrayTestDebug},
    {"asCaDebug", iocshArgInt, pvar_int_asCaDebug},
    {"atExitDebug", iocshArgInt, pvar_int_atExitDebug},
    {"boHIGHlimit", iocshArgDouble, pvar_double_boHIGHlimit},
    {"boHIGHprecision", iocshArgInt, pvar_int_boHIGHprecision},
    {"calcoutODLYlimit", iocshArgDouble, pvar_double_calcoutODLYlimit},
    {"calcoutODLYprecision", iocshArgInt, pvar_int_calcoutODLYprecision},
    {"callbackParallelThreadsDefault", iocshArgInt, pvar_int_callbackParallelThreadsDefault},
    {"dbBptNotMonotonic", iocshArgInt, pvar_int_dbBptNotMonotonic},
    {"dbQuietMacroWarnings", iocshArgInt, pvar_int_dbQuietMacroWarnings},
    {"dbRecordsAbcSorted", iocshArgInt, pvar_int_dbRecordsAbcSorted},
    {"dbRecordsOnceOnly", iocshArgInt, pvar_int_dbRecordsOnceOnly},
    {"dbTemplateMaxVars", iocshArgInt, pvar_int_dbTemplateMaxVars},
    {"dbThreadRealtimeLock", iocshArgInt, pvar_int_dbThreadRealtimeLock},
    {"debugSubAve", iocshArgInt, pvar_int_debugSubAve},
    {"devaCalcoutSoftDebug", iocshArgInt, pvar_int_devaCalcoutSoftDebug},
    {"devsCalcoutSoftDebug", iocshArgInt, pvar_int_devsCalcoutSoftDebug},
    {"eraseNDAttributes", iocshArgInt, pvar_int_eraseNDAttributes},
    {"histogramSDELprecision", iocshArgInt, pvar_int_histogramSDELprecision},
    {"interpDebug", iocshArgInt, pvar_int_interpDebug},
    {"sCalcLoopMax", iocshArgInt, pvar_int_sCalcLoopMax},
    {"sCalcPerformDebug", iocshArgInt, pvar_int_sCalcPerformDebug},
    {"sCalcPostfixDebug", iocshArgInt, pvar_int_sCalcPostfixDebug},
    {"sCalcStackHW", iocshArgInt, pvar_int_sCalcStackHW},
    {"sCalcStackLW", iocshArgInt, pvar_int_sCalcStackLW},
    {"sCalcoutRecordDebug", iocshArgInt, pvar_int_sCalcoutRecordDebug},
    {"seqDLYlimit", iocshArgDouble, pvar_double_seqDLYlimit},
    {"seqDLYprecision", iocshArgInt, pvar_int_seqDLYprecision},
    {"sseqRecDebug", iocshArgInt, pvar_int_sseqRecDebug},
    {"swaitRecordDebug", iocshArgInt, pvar_int_swaitRecordDebug},
    {"transformRecordDebug", iocshArgInt, pvar_int_transformRecordDebug},
    {NULL, iocshArgInt, NULL}
};

int cathodeSupportSim_registerRecordDeviceDriver(DBBASE *pbase)
{
    static int executed = 0;
    const char *bldTop = "/usr/local/epics/ioc";
    const char *envTop = getenv("TOP");

    if (envTop && strcmp(envTop, bldTop)) {
        printf("Warning: IOC is booting with TOP = \"%s\"\n"
               "          but was built with TOP = \"%s\"\n",
               envTop, bldTop);
    }

    if (!pbase) {
        printf("pdbbase is NULL; you must load a DBD file first.\n");
        return -1;
    }

    if (executed) {
        printf("Warning: Registration already done.\n");
    }
    executed = 1;

    registerRecordTypes(pbase, NELEMENTS(rtl), recordTypeNames, rtl);
    registerDevices(pbase, NELEMENTS(devsl), deviceSupportNames, devsl);
    registerDrivers(pbase, NELEMENTS(drvsl), driverSupportNames, drvsl);
    pvar_func_NDAttrPlotRegister();
    pvar_func_NDAttrRegister();
    pvar_func_NDCircularBuffRegister();
    pvar_func_NDCodecRegister();
    pvar_func_NDColorConvertRegister();
    pvar_func_NDFFTRegister();
    pvar_func_NDGatherRegister();
    pvar_func_NDOverlayRegister();
    pvar_func_NDPosPluginRegister();
    pvar_func_NDProcessRegister();
    pvar_func_NDROIRegister();
    pvar_func_NDROIStatRegister();
    pvar_func_NDScatterRegister();
    pvar_func_NDStatsRegister();
    pvar_func_NDStdArraysRegister();
    pvar_func_NDTimeSeriesRegister();
    pvar_func_NDTransformRegister();
    pvar_func_arrInitialize();
    pvar_func_arrayTestRegister();
    pvar_func_asSub();
    pvar_func_asynInterposeDelayRegister();
    pvar_func_asynInterposeEchoRegister();
    pvar_func_asynInterposeEosRegister();
    pvar_func_asynInterposeFlushRegister();
    pvar_func_asynMotorControllerRegister();
    pvar_func_asynRegister();
    pvar_func_dbndInitialize();
    pvar_func_decInitialize();
    pvar_func_editSseqRegistrar();
    pvar_func_interpRegister();
    pvar_func_motorRegister();
    pvar_func_motorSimDriverRegister();
    pvar_func_motorSimRegister();
    pvar_func_motorUtilRegister();
    pvar_func_parseRegister();
    pvar_func_prosilicaRegister();
    pvar_func_simDetectorRegister();
    pvar_func_subAveRegister();
    pvar_func_syncInitialize();
    pvar_func_tsInitialize();
    pvar_func_register_func_myAttrFunct1();
    pvar_func_register_func_myTimeStampSource();
    pvar_func_register_func_rebootProc();
    pvar_func_register_func_scanMon();
    pvar_func_register_func_scanMonInit();
    iocshRegisterVariable(vardefs);
    return 0;
}

/* cathodeSupportSim_registerRecordDeviceDriver */
static const iocshArg rrddArg0 = {"pdbbase", iocshArgPdbbase};
static const iocshArg *rrddArgs[] = {&rrddArg0};
static const iocshFuncDef rrddFuncDef =
    {"cathodeSupportSim_registerRecordDeviceDriver", 1, rrddArgs};
static void rrddCallFunc(const iocshArgBuf *)
{
    cathodeSupportSim_registerRecordDeviceDriver(*iocshPpdbbase);
}

} // extern "C"

/*
 * Register commands on application startup
 */
static int Registration() {
    iocshRegisterCommon();
    iocshRegister(&rrddFuncDef, rrddCallFunc);
    return 0;
}

static int done EPICS_UNUSED = Registration();