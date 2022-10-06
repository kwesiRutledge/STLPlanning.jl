"""
node.jl
Description:
    Type and function definitions related to node objects.
"""

# ==============
# Abstract Types
# ==============

abstract type Node end

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

# function ConjunctionNode(childrenIn::Vector{Union{Node,Float64}})::ConjunctionNode
#     return ConjunctionNode(childrenIn,[],[],"")
# end

# function ConjunctionNode(childrenIn::Vector{Float64})::ConjunctionNode
#     return ConjunctionNode(childrenIn,[],[],"")
# end

function ConjunctionNode(childrenIn)::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"")
end

function ConjunctionNode(childrenIn::Vector{Node})::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"")
end

function ConjunctionNode(childrenIn::Vector{Float64},constraintIn::Vector{Float64})::ConjunctionNode
    return ConjunctionNode(childrenIn,constraintIn,[],"")
end

# function DisjunctionNode(childrenIn::Vector{Union{Node,Float64}})::DisjunctionNode
#     return DisjunctionNode(childrenIn,[],[],"")
# end

# function DisjunctionNode(childrenIn::Vector{Float64})::DisjunctionNode
#     return DisjunctionNode(childrenIn,[],[],"")
# end

function DisjunctionNode(childrenIn)::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"")
end

function DisjunctionNode(childrenIn::Vector{Node})::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"")
end