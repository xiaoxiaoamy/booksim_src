#! /usr/bin/perl

open (INFILE, "cmesh_rate_sample");

$file_out = $ARGV[0];
$rate = $ARGV[1];
#open (OUTFILE, ">>$file_out");
#print $file_out;

$line;

  while ($line = <INFILE>){
#    print "aaa\n";
    if ($line =~/injection_rate = .*/s){
      close (OUTFILE);
      open (OUTFILE, ">>$file_out");
      print OUTFILE "injection_rate = $rate;\n";
#      print "injection_rate = $rate;\n";
    } else {
      close (OUTFILE);
      open (OUTFILE, ">>$file_out");
      print OUTFILE $line;
#      print $line;
    }
  }


close INFILE;
close OUTFILE;
