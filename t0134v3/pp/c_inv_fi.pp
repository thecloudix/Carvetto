POST_NAME = "invfi"
UNITS = "INCHES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.1]
VAR INV = [FI|A| F|1.2]
begin HEADER
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G93 G1[X][Y][Z][FI]"
end
begin FOOTER
"M5"
end
