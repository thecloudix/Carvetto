POST_NAME = "rapidf"
UNITS = "MM"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.0]
begin HEADER
"M3[S]"
end
begin RAPID_MOVE
"G0[X][Y][Z][F]"
end
begin FEED_MOVE
"G1[X][Y][Z][F]"
end
begin FOOTER
"M5[S]"
end
