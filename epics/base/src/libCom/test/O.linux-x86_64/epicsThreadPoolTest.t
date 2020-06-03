#!/usr/bin/perl

use strict;
use Cwd 'abs_path';

$ENV{HARNESS_ACTIVE} = 1 if scalar @ARGV && shift eq '-tap';
$ENV{TOP} = abs_path($ENV{TOP}) if exists $ENV{TOP};

if ($^O eq 'MSWin32') {
    # Use system on Windows, exec doesn't work the same there and
    # GNUmake thinks the test has finished as soon as Perl exits.
    system('./epicsThreadPoolTest') == 0 or die "Can't run epicsThreadPoolTest: $!\n";
}
else {
    exec './epicsThreadPoolTest' or die "Can't run epicsThreadPoolTest: $!\n";
}
