#
#  Copyright 2017 by Texas Instruments Incorporated.
#  
#

use CCS_SCRIPTING_PERL;
use Getopt::Std;

getopts('cdp:');
$proj = shift(@ARGV);

$ccs = new CCS_SCRIPTING_PERL::CCS_Scripting();

print "starting up CCS ...\n";
$ccs->CCSOpenNamed('*', '*', 1);

if ($proj) {
#    print "closing current project ...\n";
#    $err = $ccs->ProjectClose();

    print "loading $proj ...\n";
    $ccs->ProjectOpen($proj);

    $gel = $proj;
    $gel =~ s/\.pjt$/.gel/;
    $gel =~ s/\\/\//g;
    print "checking for GEL file $gel ...\n";
    if (-r $gel) {
        $expr = sprintf("GEL_LoadGel(\"%s\")", $gel);
        $ccs->TargetEvalExpression($expr);
        print "loading dependent packages ...\n";
        $ccs->TargetEvalExpression("XDC_open()");
    }
}
#
#  @(#) ti.targets; 1, 0, 3,1; 7-27-2017 11:47:28; /db/ztree/library/trees/xdctargets/xdctargets-o04/src/ xlibrary

#

