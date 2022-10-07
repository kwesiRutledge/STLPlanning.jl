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

#=
# Helper Functions 
=#

"""
handleSpecTree
Description:
    This function appears to create optimization AND expressions for each of the nodes in the tree.
"""
function handleSpecTree(specificationTreeRoot::Node,pwl_curve::Vector{Tuple{Vector{VariableRef},VariableRef}},bloat_factor::Float64,size)
    # Recursion Starts Here (Recurse for each dependency to this node)
    for dependency in specificationTreeRoot.Children
        handleSpecTree(dependency, pwl_curve, bloat_factor,size)
    end

    # Input Processing
    if length(specificationTreeRoot.Zs) == length(pwl_curve)-1
        return
    elseif length(specificationTreeRoot.Zs) > 0
        throw(ErrorException("Incomplete Zs!"))
    end

    # Now, use the expressions provided by our children to create constraints this node (specificationTreeRoot).
    if specificationTreeRoot.Operator == "mu"
        specificationTreeRoot.Zs = [ mu(i) ]
    end
    

end