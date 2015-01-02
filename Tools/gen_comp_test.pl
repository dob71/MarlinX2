#!/usr/bin/perl

# Print start/end files (i.e. start/end files from Skeinforge profile)
$fstart = "all_start_x2v3.gcode";
$fend = "all_end_x2v3.gcode";

# Filament diameter
$f_d = 2.957;
# Layer height
$l_h = 0.23;
# Layer width
$l_w = 0.58;

# Parameters of the pattern (printing right, up, left, up, right, up ...)
$x_from = 25.0;
$x_to = 175.0;
$y_from = 50.0;
$y_to = 150.0;
$y_step = 10.0;

# Feed rate (mm/sec) for x-lines (changes by x-step)
# Feed rate (mm/sec) for y-lines (changes by y-step)
$y_f_from = 110.0;
$y_f_step = -10.0;
$x_f_from = 10.0;
$x_f_step = 10.0;

# Automatically perform retract/restore for travel moves in firmware
$builtin_retr = 2.2; 

# Overcompensation value for M340 O# command
$overcomp = 0.2;

# Compensation table ($builtin_retr adjusts it for auto retract/restore)
$comp{0.05} = 0.525;
$comp{0.10} = 0.735;
$comp{0.25} = 1.400;
$comp{0.50} = 2.850;
$comp{1.00} = 3.800;

# return filament distance (when starting, after init file retract)
# make it return all the filament retracted minus the retract length 
# used for printing (i.e. $e_retr or $builtin_retr depending on either 
# you are trying regular or automatic retract).
$e_start = 0.0; #2.7;
# return filament feedrate mm/min
$e_start_f = 2700.0;
# travel feedrate mm/min
$t_f = 75.0;
# turn on testing with travel
$travel = 1;

# retract between speed changes, mm and retract feedrate in mm/s
# (this turns on normal retract/return, i.e. done in G-code)
$e_retr = 0.0;
$e_retr_fr = 17.0;

# ====================== end setup ===========================

$pi = 3.1415926;

# layer cross section area (rectagular center, semispere left and right)
$lc_area = ($l_h * ($l_w - $l_h)) + ($pi * $l_h * l_h / 4.0);
# filament cross section area (circle)
$fc_area = $pi * $f_d * $f_d / 4.0;
# E rate
$e_r = $lc_area / $fc_area;

# include start gcode
open(F, '<', $fstart) or die $!;
while(<F>) {
   s/;.*//;
   s/^\s+|\s+$//g;
   if(length($_) <= 0) {
      next;
   }
   print $_,"\n";
}
close(F);

# feedrates for X and y moves to start with
$x_f = $x_f_from;
$y_f = $y_f_from;

# Set up compensation table
$pos = 0;
for my $key (sort keys(%comp)) {
  print "M340 P$pos S$key C$comp{$key}\n";
  $pos++;
}
print "M340 O$overcomp\n";
print "M340 R$builtin_retr\n";

# position at the beginning
$x = $x_to;
$y = $y_from;
$e = $e_start;
printf "G1 X$x Y$y Z$l_h F%0.4f\n", $t_f * 60;
printf "G1 E$e F$e_start_f\n";

# generate the pattern
for($y = $y_from; $y <= $y_to; $y += $y_step) {
  if($y != $y_from) {
    if($e_retr > 0.0) {
      printf "G1 E%0.4f F%0.4f\n", $e-$e_retr, $e_retr_fr*60;
    }
    if($travel) {
      if($x == $x_to) {
        $x = $x_from;
      } else {
        $x = $x_to;
      }
      printf "G1 X%0.4f Y%0.4f F%0.4f\n", $x, $y, $t_f*60.0;
      $y_old = $y;
      $y = $y - $y_step + $l_w;
    }
    if($e_retr > 0.0) {
      printf "G1 E%0.4f F%0.4f\n", $e, $e_retr_fr*60;
    }

    $e += $e_r * abs($y-$y_old);
    printf "G1 X%0.4f Y%0.4f E%0.4f F%0.4f ; e-speed: %f dst: %f\n", $x, $y, $e, $y_f*60.0, $e_r*$y_f, abs($y-$y_old);

    if($e_retr > 0.0) {
      printf "G1 E%0.4f F%0.4f\n", $e-$e_retr, $e_retr_fr*60;
    }
    if($travel) {
      if($x == $x_to) {
        $x = $x_from;
      } else {
        $x = $x_to;
      }
      $y = $y + $y_step - $l_w;
      printf "G1 X%0.4f Y%0.4f F%0.4f\n", $x, $y, $t_f*60.0;
    }
    if($e_retr > 0.0) {
      printf "G1 E%0.4f F%0.4f\n", $e, $e_retr_fr*60;
    }
  }
  # going back and forth on X
  if($x == $x_to) {
    $x = $x_from;
  } else {
    $x = $x_to;
  }
  $e += $e_r * ($x_to - $x_from);
  printf "G1 X%0.4f Y%0.4f E%0.4f F%0.4f ; e-speed: %f dst: %f\n", $x, $y, $e, ,$x_f*60.0, $e_r*$x_f, ($x_to - $x_from);

  # Save current Y
  $y_old = $y;

  # Adjust the feedrates
  $x_f += $x_f_step;
  $y_f += $y_f_step;
}

# Clean compensation table
$pos = 0;
for my $key (sort keys(%comp)) {
  print "M340 P$pos S0 C0\n";
  $pos++;
}

# end file
open(F, '<', $fend) or die $!;
while(<F>) {
   s/;.*//;
   s/^\s+|\s+$//g;
   if(length($_) <= 0) {
      next;
   }
   print $_,"\n";
}
close(F);

