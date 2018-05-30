# Time of obstacle observation, coordinate frame ID
Header header

# The coefficients representing a polynomial line
# The degree of the polynomial is determined by the number of coefficients
float64[] coefficients

# What the polynomial is in terms of / with respect to
uint8 DEPENDENT_ON_X=0 # (ie. y(x) )
uint8 DEPENDENT_ON_Y=1 # (ie. x(y) )
uint8 dependent_on

# The min and max values for the polynomial line
# These are in terms of the DEPENDENT variable
# (ie. x_min/x_max for y(x), y_min/y_max for x(y) )
# this is what makes it a line segment, rather then a never-ending line
float64 max
float64 min
