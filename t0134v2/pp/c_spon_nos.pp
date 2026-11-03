POST_NAME = "spon"
UNITS = "MM"
VAR FEED_RATE = [F|C| F|1.0]
begin HEADER
"M3"
end
begin TOOLCHANGE
"M5"
"M0"
end
begin SPINDLE_ON
"M3"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z][F]"
end
begin FOOTER
"M5"
end
