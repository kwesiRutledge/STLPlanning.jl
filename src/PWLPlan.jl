"""
PWLPlan.jl
Description:
    An implementation of some of the algorithms presented in a file of the same name from STLPlanning.
    Should contain:
        - Implementations of the STLTree node objects.
"""

# ==============
# Abstract Types
# ==============

abstract type Node end

# =======================
# Regular Types (Structs)
# =======================

struct ConjunctionNode <: Node
    Children::Vector{Node,Float64}
    Constraints::Any
end

"""
DisjunctionNode
Description:
    An object that defines the disjunction between all formulas
    that are represented as its children.
"""
struct DisjunctionNode <: Node
    Children::Vector{Node,Float64}
    Constraints::Any
end

# =========
# Functions
# =========

"""
noIntersection
Description:

From STLPlanning:
    z = 1 iff [a,b] and [c,d] has no intersection
    b < c or d < a
Notes:
    I suspect that a,b,c, and d are all continuous variables in the problem.
    In our case, I think that these may be constants but could also be variables(?)
"""
function noIntersection(a,b,c,d)
    return DisjunctionNode([c-b-EPS,a-d-EPS],[])
end

"""
hasIntersection
Description:

From STLPlanning:
    z = 1 iff [a,b] and [c,d] has an intersection
    b >= c AND d >= a
Notes:
    I suspect that a,b,c, and d are all continuous variables in the problem.
    In our case, I think that these may be constants but could also be variables(?)
"""
function hasIntersection(a,b,c,d)
    return ConjunctionNode([b-c,d-a],[])
end

"""
always(i,a,b,zphis,PWL)
Description:
    This function should return a subtree corresponding to the formula always.
Variables:
    segment_index - The index of the robot paths.
                    Each path will be a piece-wise linear curve.
    a - Starting time index where the always must hold.
    b - Ending time index where the always formula must hold.
    zphis - ??? What is this?
    curve - This is an array of arrays defining the curve that the robot
            plans.
"""
function always(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Vector{Float64}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][1]
    t_i_1 = pwl_curve[segment_index+1][1]
    conjunctions = Vector{Node}([])

    # Loop
    for j in range(1,length(pwl_curve)-1)
        t_j = PWL[j][1]
        t_j_1 = PWL[j+1][1]
        append!(conjunctions,DisjunctionNode([noIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]]))
    end
    
end