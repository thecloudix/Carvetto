POST_NAME = "finormal"
UNITS = "INCHES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.1]
VAR INV = [FI|A| F|1.0|0.5]
begin HEADER
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z][F](inv [FI])"
end
begin PLUNGE_MOVE
"G93 G1[X][Y][Z][FI]"
end
begin CW_ARC_MOVE
"G93 G2[X][Y][Z][I][J][FI]"
end
begin CCW_ARC_MOVE
"G93 G3[X][Y][Z][I][J][FI]"
end
begin FOOTER
"M5"
end
