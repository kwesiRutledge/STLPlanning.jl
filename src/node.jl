"""
node.jl
Description:
    Type and function definitions related to node objects.
"""

using JuMP, Gurobi

# ==============
# Abstract Types
# ==============

abstract type Node end

# =======================
# Regular Types (Structs)
# =======================

# A strange grab bag of data that is associated with
# different nodes. Not all fields need to be defined.
struct NodeInfo
    A
    b
    Name::String
    IntervalStart::Float64
    IntervalEnd::Float64
end

mutable struct ConjunctionNode <: Node
    Children::Vector{Any}
    Constraints::Any
    Zs
    Operation::String
    Info::NodeInfo
end

"""
DisjunctionNode
Description:
    An object that defines the disjunction between all formulas
    that are represented as its children.
"""
mutable struct DisjunctionNode <: Node
    Children::Vector{Any}
    Constraints::Any
    Zs
    Operation::String
    Info::NodeInfo
end

mutable struct BasicNode <: Node
    Children::Vector{Any}
    Constraints::Any
    Zs
    Operation::String
    Info::NodeInfo
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
    return ConjunctionNode(childrenIn,[],[],"and",NodeInfo())
end

function ConjunctionNode(childrenIn::Vector{Node})::ConjunctionNode
    return ConjunctionNode(childrenIn,[],[],"and",NodeInfo())
end

function ConjunctionNode(childrenIn,constraintIn)::ConjunctionNode
    return ConjunctionNode(childrenIn,constraintIn,[],"and",NodeInfo())
end

function ConjunctionNode(childrenIn,constraintIn,Zs,ni_In::NodeInfo)::ConjunctionNode
    return ConjunctionNode(childrenIn,constraintIn,Zs,"and",ni_In)
end

# function DisjunctionNode(childrenIn::Vector{Union{Node,Float64}})::DisjunctionNode
#     return DisjunctionNode(childrenIn,[],[],"")
# end

# function DisjunctionNode(childrenIn::Vector{Float64})::DisjunctionNode
#     return DisjunctionNode(childrenIn,[],[],"")
# end

function DisjunctionNode(childrenIn)::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"or",NodeInfo())
end

function DisjunctionNode(childrenIn::Vector{Node})::DisjunctionNode
    return DisjunctionNode(childrenIn,[],[],"or",NodeInfo())
end

function DisjunctionNode(childrenIn,constraintIn,Zs,ni_In::NodeInfo)::DisjunctionNode
    return DisjunctionNode(childrenIn,constraintIn,Zs,"or",ni_In)
end

"""
BasicNode Constructors
"""
function BasicNode(operationIn::String, ni_in::NodeInfo)::BasicNode
    return BasicNode([],[],[],operationIn,ni_in)
end

function BasicNode(operationIn::String, children_in, ni_in::NodeInfo)::BasicNode
    return BasicNode(children_in,[],[],operationIn,ni_in)
end

"""
NodeInfo Constructors
"""
function NodeInfo()
    return NodeInfo([],[],"",-1,-1)
end

function NodeInfo(name::String)
    return NodeInfo([],[],name,-1,-1)
end

function NodeInfo(A::Matrix{Float64},b::Vector{Float64})
    return NodeInfo(A,b,"",-1,-1)
end

function NodeInfo(intervalStartTime::Float64,intervalEndTime::Float64)
    return NodeInfo([],[],"",intervalStartTime,intervalEndTime)
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
        return # I think this means that the Zs were already constructed(?)
    elseif length(specificationTreeRoot.Zs) > 0
        throw(ErrorException("Incomplete Zs!"))
    end

    # println("Current Node")
    # println(specificationTreeRoot)

    # Now, use the expressions provided by our children to create constraints this node (specificationTreeRoot).
    if specificationTreeRoot.Operation == "mu"
        for i in range(1,stop=length(pwl_curve)-1)
            push!( 
                specificationTreeRoot.Zs , 
                mu(
                    i, 
                    pwl_curve, 
                    specificationTreeRoot.Info.A, 
                    specificationTreeRoot.Info.b,
                    bloat_factor
                )
            )
        end
    elseif specificationTreeRoot.Operation == "negmu"
        for i in range(1,stop=length(pwl_curve)-1)
            println(string("negmu at segment #",i))
            # println(string("[negmu] i = ",i))
            push!(
                specificationTreeRoot.Zs ,
                negmu(
                    i, 
                    pwl_curve, 
                    specificationTreeRoot.Info.A, 
                    specificationTreeRoot.Info.b,
                    bloat_factor + size
                )
            )
        end
    elseif specificationTreeRoot.Operation == "and"
        for segment_index in range(1,stop=length(pwl_curve)-1)
            conjunction_at_i_zs = []
            for child in specificationTreeRoot.Children
                child_z_i = child.Zs[segment_index]
                push!(conjunction_at_i_zs,child_z_i)
            end
            push!(
                specificationTreeRoot.Zs ,
                ConjunctionNode(conjunction_at_i_zs)
            )
        end
    elseif (specificationTreeRoot.Operation == "or") || ( specificationTreeRoot.Operation == "∧" )
        for segment_index in range(1,stop=length(pwl_curve)-1)
            disjunction_at_i_zs = []
            for child in specificationTreeRoot.Children
                child_z_i = child.Zs[segment_index]
                push!(disjunction_at_i_zs,child_z_i)
            end
            push!(
                specificationTreeRoot.Zs ,
                DisjunctionNode(disjunction_at_i_zs)
            )
        end
    elseif (specificationTreeRoot.Operation == "F") || (specificationTreeRoot.Operation == "♢")
        for segment_index in range(1,stop=length(pwl_curve)-1)
            push!(
                specificationTreeRoot.Zs ,
                eventually(
                    segment_index,
                    specificationTreeRoot.Info.IntervalStart,
                    specificationTreeRoot.Info.IntervalEnd,
                    specificationTreeRoot.Children[1].Zs, # Why use just the first child? Their should only be 1 child for an eventually node. It represents the formula we want to "eventually" hold.
                    pwl_curve
                )
            )
        end
    elseif (specificationTreeRoot.Operation == "A") || (specificationTreeRoot.Operation == "□")
        for segment_index in range(1,stop=length(pwl_curve)-1)
            println("specificationTree.Children[1].Zs =")
            println(length(specificationTreeRoot.Children))
            println(specificationTreeRoot.Children[1].Zs)
            push!(
                specificationTreeRoot.Zs ,
                always(
                    segment_index,
                    specificationTreeRoot.Info.IntervalStart,
                    specificationTreeRoot.Info.IntervalEnd,
                    specificationTreeRoot.Children[1].Zs, # Why use just the first child? There should only be 1 child for an eventually node. It represents the formula we want to "eventually" hold.
                    pwl_curve
                )
            )
        end
    else
        throw(
            ErrorException(string("The operation for the current node (", specificationTreeRoot.Operation ,") is unexpected!"))
        )
    end

end

"""
gen_CDTree_constraints
Description:

"""
function gen_CDTree_constraints(jump_model, rootNode::Node)
    println("Current node: ")
    println(rootNode)
    println(rootNode.Children)
    println("")

    if length(rootNode.Children) == 0
        # Return the node itself
        return Vector([rootNode])
    else 
        if length(rootNode.Constraints) > 0
            # Constraints have already been created.
            return rootNode.Constraints
        end

        # Create the constraints for the dependencies
        dependency_constraints = []
        child_index = 0
        for child in rootNode.Children # Creating constraints from each dependency
            # println(string("child #",child_index))
            child_index = child_index + 1
            append!(
                dependency_constraints, 
                gen_CDTree_constraints(jump_model,child)
            )
        end

        #
        Zs = []
        # println(dependency_constraints)
        println("rootNode is still:")
        println(rootNode)
        for dep_con_index in range(1,stop=length(dependency_constraints))
            dep_cons = dependency_constraints[dep_con_index]
            
            println("dep_cons = ")
            println(dep_cons)
            println(string("isa(dep_con,Node) = ",isa(dep_cons,Node)))
            println(string("isa(rootNode,Node) = ",isa(rootNode,Node)))

            interm_constraints = []
            if isa( rootNode , DisjunctionNode )
                println("Found a disjunction node!")
                z = @variable(
                    jump_model,
                    binary=true
                )
                push!(Zs,z)
                
                #interm_constraints = Vector{AffExpr}([])
                println("Iterating through dep_cons...")
                for constraint in dep_cons
                    println(constraint)
                    println(constraint + M * (1 - z))
                    push!(interm_constraints , constraint + M * (1 - z))
                end
                println(interm_constraints)
                println("done with dep_cons")
            end
            
            if isa(dep_cons,AffExpr)
                interm_constraints = [dep_cons]
            else
                interm_constraints = dep_cons
            end

            

            append!(rootNode.Constraints,interm_constraints)
            
            println("rootNode.Constraints")
            println(rootNode.Constraints)
            println("")
        end

        if length(Zs) > 0
            push!(rootNode.Constraints,sum(Zs)-1)
        end

        # println("Completed handleSpecTree!")

        return rootNode.Constraints
    end
    
end

function gen_CDTree_constraints(jump_model, rootNode::Union{AffExpr,VariableRef})
    return Vector([rootNode])
end

"""
add_CDTree_Constraints
Description:
    Adds the constraints of the CDTree to the JuMP model.
"""
function add_CDTree_Constraints(jump_model, rootNode::Node)
    constrs = gen_CDTree_constraints(jump_model, rootNode::Node)
    # println("All constraints!")
    for con in constrs
        @constraint(jump_model, con >= 0.0)
        # println(con)
    end
end

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