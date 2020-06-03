/* THIS IS A GENERATED FILE. DO NOT EDIT! */
/* Generated from ../O.Common/filterTest.dbd */

#include <string.h>

#include "compilerDependencies.h"
#include "epicsStdlib.h"
#include "iocsh.h"
#include "iocshRegisterCommon.h"
#include "registryCommon.h"

extern "C" {

epicsShareExtern rset *pvar_rset_arrRSET, *pvar_rset_xRSET;

typedef int (*rso_func)(dbRecordType *pdbRecordType);
epicsShareExtern rso_func pvar_func_arrRecordSizeOffset,
    pvar_func_xRecordSizeOffset;

static const char * const recordTypeNames[] = {
    "arr", "x"
};

static const recordTypeLocation rtl[] = {
    {pvar_rset_arrRSET, pvar_func_arrRecordSizeOffset},
    {pvar_rset_xRSET, pvar_func_xRecordSizeOffset}
};

typedef void (*reg_func)(void);
epicsShareExtern reg_func pvar_func_arrInitialize,
    pvar_func_dbndInitialize, pvar_func_decInitialize,
    pvar_func_syncInitialize, pvar_func_tsInitialize;

int filterTest_registerRecordDeviceDriver(DBBASE *pbase)
{
    static int executed = 0;
    const char *bldTop = "/home/fphysics/whitegr/facet-sw/epics/base";
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
    pvar_func_arrInitialize();
    pvar_func_dbndInitialize();
    pvar_func_decInitialize();
    pvar_func_syncInitialize();
    pvar_func_tsInitialize();
    return 0;
}

/* filterTest_registerRecordDeviceDriver */
static const iocshArg rrddArg0 = {"pdbbase", iocshArgPdbbase};
static const iocshArg *rrddArgs[] = {&rrddArg0};
static const iocshFuncDef rrddFuncDef =
    {"filterTest_registerRecordDeviceDriver", 1, rrddArgs};
static void rrddCallFunc(const iocshArgBuf *)
{
    filterTest_registerRecordDeviceDriver(*iocshPpdbbase);
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
