# Time of obstacle observation, coordinate frame ID
Header header

# The coefficients representing a polynomial line
# The degree of the polynomial is determined by the number of coefficients
float64[] coefficients

# The min and max x values for the polynomial line
# this is what makes it a line segment, rather then a never-ending line
float64 x_max
float64 x_min