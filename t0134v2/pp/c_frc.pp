POST_NAME = "frc"
UNITS = "INCHES"
VAR SPINDLE_SPEED = [S|A| S|1.0]
VAR FEED_RATE = [F|A| F|1.2]
VAR FC = [FC|A| F|1.2]
VAR FP = [FP|A| F|1.2]
begin HEADER
"(feeds [FC] [FP])"
"M3[S]"
end
begin FEED_RATE_CHANGE
"F[F]"
end
begin RAPID_MOVE
"G0[X][Y][Z]"
end
begin FEED_MOVE
"G1[X][Y][Z]"
end
begin FOOTER
"M5"
end
