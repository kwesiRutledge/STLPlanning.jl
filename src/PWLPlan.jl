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

const EPS = 0.00001

# =======================
# Regular Types (Structs)
# =======================

# struct Node
#     Operator::String
#     Children::Vector{Union{Node,Float64}}
#     Zs::Any
#     Info::String
# end

mutable struct ConjunctionNode <: Node
    Children::Vector{Union{Node,Float64}}
    Constraints::Any
    Zs
    Info::String
end

"""
DisjunctionNode
Description:
    An object that defines the disjunction between all formulas
    that are represented as its children.
"""
mutable struct DisjunctionNode <: Node
    Children::Vector{Union{Node,Float64}}
    Constraints::Any
    Zs
    Info::String
end

# ==================
# Quick Constructors
# ==================

function ConjunctionNode(childrenIn::Vector{Union{Node,Float64}})::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"")
end

function ConjunctionNode(childrenIn::Vector{Float64})::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"")
end

function ConjunctionNode(childrenIn::Vector{Node})::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"")
end

function ConjunctionNode(childrenIn::Vector{Float64},constraintIn::Vector{Float64})::ConjunctionNode
    return ConjunctionNode(childrenIn,constraintIn,[],"")
end

function DisjunctionNode(childrenIn::Vector{Union{Node,Float64}})::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"")
end

function DisjunctionNode(childrenIn::Vector{Float64})::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"")
end

function DisjunctionNode(childrenIn::Vector{Node})::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"")
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
function noIntersection(a,b,c,d)::DisjunctionNode
    return DisjunctionNode([c-b-EPS,a-d-EPS])
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
function hasIntersection(a,b,c,d)::ConjunctionNode
    return ConjunctionNode([b-c,d-a])
end

"""
always(segment_index,a,b,zphis,pwl_curve)
Description:
    This function should return a subtree corresponding to the formula 
    segment i satisfies always zphis between a and b.
Variables:
    segment_index - The index of the robot paths.
                    Each path will be a piece-wise linear curve.
    a - Starting time index where the always must hold.
    b - Ending time index where the always formula must hold.
    zphis - ??? What is this?
            Perhaps this is the value of z over the course of the time horizon defined
            by PWL.
    curve - This is an array of arrays defining the curve that the robot
            plans.
"""
function always(segment_index::Int,a::Float64,b::Float64,zphis::Vector{Float64},pwl_curve::Vector{Vector{Float64}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][1]
    t_i_1 = pwl_curve[segment_index+1][1]
    conjunctions = Vector{Node}([])

    # Loop
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][1]
        t_j_1 = pwl_curve[j+1][1]
        push!(conjunctions,
            DisjunctionNode(
                Vector{Union{Node,Float64}}([noIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]])
                )
            )
    end
    
    # Create node
    return ConjunctionNode(conjunctions)

end

"""
eventually(segment_index,a,b,zphis,pwl_curve)
Description:
    This function should return a subtree corresponding to the formula eventually.
Variables:
    segment_index - The index of the robot paths.
                    Each path will be a piece-wise linear curve.
    a - Starting time index where the always must hold.
    b - Ending time index where the always formula must hold.
    zphis - ??? What is this?
    curve - This is an array of arrays defining the curve that the robot
            plans.
"""
function eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Vector{Float64}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][1]
    t_i_1 = pwl_curve[segment_index+1][1]

    z_interval_width = b - a - (t_i_1 - t_i) - EPS

    # Loop
    disjunctions = Vector{Node}([])
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = PWL[j][1]
        t_j_1 = PWL[j+1][1]
        push!(disjunctions,ConjunctionNode([hasIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]]))
    end
    
    # Create node
    return ConjunctionNode([z_interval_width,DisjunctionNode(disjunctions)])
end

"""
bounded_eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Vector{Float64}})
Description:

"""

"""
clearSpecTree
Description:
    Removes all of the "z" s from each node in the tree.
"""
function clearSpecTree(spec::Node)
    # Iterate through each dependency of the tree
    for node in spec.Children
        clearSpecTree(node)
    end
    spec.Zs = []
end

function clearSpecTree(spec::Float64)
    # We should not do anything if the node is a float and
    # not a Node

    # Do nothing

end

"""
plan(x0s)
Description:
    The planning function for the STLPlanning python repository.
"""
function plan(x0s::Vector{Vector{Float64}},specificationTree::Vector{Node},limits=[],num_segs::Integer=-1,tasks=[],vmax::Float64=3.0,MIPGap::Float64=1e-4,max_segs::Integer=-1, tmax::Float64=-1.0, hard_goals=[], size=0.11*4/2)
    # Input Processing
    if num_segs == -1 
        min_segs = 1
        if max_segs == -1 
            throw(DomainError("The maximum number of segments must be given if the user doesn't want to specify the exact number of segments."))
        end
    else
        min_segs = num_segs
        max_segs = num_segs
    end

    # Iterate through the many potential number of segments
    for num_segs in range(min_segs,stop=max_segs)
        for spec in specs
            clearSpecTree(spec)
        end
    end
end