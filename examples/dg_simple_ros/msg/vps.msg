####################
# vps
####################

# Common header
Header header

# matched streetview ID
uint64 id

# detected poi_xy and relative pose(r,pi)
float64 x
float64 y
float64 rel_r
float64 rel_pi

# confidence (0 ~ 1)
float64 confidence

# processing time (in seconds)
float64 processingtime

# timestamp (in seconds)
float64 timestamp
