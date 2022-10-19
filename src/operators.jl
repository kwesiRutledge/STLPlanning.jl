"""
operators.jl
Description:
    This file should contain the functions we use to represent various
    Signal Temporal Logic (STL) operators.
"""

using LinearAlgebra

#===============
Helper Functions
===============#

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

#==============================
Signal Temporal Logic Operators
==============================#

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
function always(segment_index::Int,a::Float64,b::Float64,zphis::Vector{Float64},pwl_curve::Vector{Tuple{Vector{Float64},Float64}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]
    conjunctions = Vector{Node}([])

    # Loop
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        push!(conjunctions,
            DisjunctionNode(
                Vector{Union{Node,Float64}}([noIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]])
                )
            )
    end
    
    # Create node
    return ConjunctionNode(conjunctions)

end

function always(segment_index::Int,a::Float64,b::Float64,zphis::Vector{Any},pwl_curve::Vector{Tuple{Vector{VariableRef},VariableRef}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]
    conjunctions = Vector{Node}([])

    # Loop
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        push!(conjunctions,
            DisjunctionNode(
                Vector{Union{Node,Float64,VariableRef}}([noIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]])
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
function eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Tuple{Vector{Any},Any}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]

    z_interval_width = b - a - (t_i_1 - t_i) - EPS

    # Loop
    disjunctions = Vector{Node}([])
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        push!(disjunctions,ConjunctionNode([hasIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]]))
    end
    
    # Create node
    return ConjunctionNode([z_interval_width,DisjunctionNode(disjunctions)])
end

function eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Tuple{Vector{VariableRef},VariableRef}})
    # Get The start times of each pwl_curve
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]

    z_interval_width = b - a - (t_i_1 - t_i) - EPS

    # Loop
    disjunctions = Vector{Node}([])
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        push!(disjunctions,ConjunctionNode([hasIntersection(t_j,t_j_1, t_i + a , t_i_1 + b),zphis[j]]))
    end
    
    # Create node
    return ConjunctionNode([z_interval_width,DisjunctionNode(disjunctions)])
end

"""
bounded_eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve::Vector{Vector{Float64}})
Description:
    This function is the bounded eventually operator <>_{[a,b],T} phi
    which attempts to satisfy phi within some time interval [a,b] AND
    before the absolute time of T.
Variables:
    segment_index - The index of the robot paths.
                    Each path will be a piece-wise linear curve.
    a - Starting time index where the always must hold.
    b - Ending time index where the always formula must hold.
    zphis - Current hypothesis is that these are expressions for the formula
            phi at each time interval defined by pwl_curve.
            If a single one of them can be shown to be true for the current segment,
            then... ?
    curve - This is an array of arrays defining the curve that the robot
            plans.
"""
function bounded_eventually(segment_index::Int,a::Float64,b::Float64,zphis,pwl_curve,tmax::Float64)::DisjunctionNode
    # Get the start and end times of segment segment_index
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]

    z_intervalWidth = b-a-(t_i_1-t_i)-EPS
    
    # Create loop of disjunctions.
    disjunctions = Vector{Node}([])
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        push!(disjunctions,ConjunctionNode([hasIntersection(t_j, t_j_1, t_i_1 + a, t_i + b), zphis[j]]))
    end

    return DisjunctionNode([
        ConjunctionNode([z_intervalWidth, DisjunctionNode(disjunctions)]),
        t_i+b-tmax-EPS
        ])

end

"""
until
Description:
    This function creates an until relationship between the zphi Variables
    and the zphi2 variables.
    Symbolically,
        phi_1 U phi_2

Variables:
    a - Starting time index where the always must hold.
    b - Ending time index where the always formula must hold.
    zphis1 - Current hypothesis is that these are expressions for the formula
            phi being satisfied W.R.T. segment component segment_index 
            at each time interval defined by pwl_curve.
    zphis2 - Current hypothesis is that these are expressions for the formula
            phi2 being satisfied W.R.T. segment component segment_index
            at each time interval defined by pwl_curve.
    curve - This is an array of arrays defining the curve that the robot
            plans.
"""
function until(segment_index::Int,a::Float64,b::Float64, zphi1s, zphi2s, pwl_curve)
    t_i = pwl_curve[segment_index][2]
    t_i_1 = pwl_curve[segment_index+1][2]

    z_intervalWidth = b - a - (t_i_1-t_i) - EPS
    disjunctions = Vector{Node}([])
    for j in range(1,stop=length(pwl_curve)-1)
        t_j = pwl_curve[j][2]
        t_j_1 = pwl_curve[j+1][2]
        conjunctions = [hasIntersection(t_j, t_j_1, t_i_1 + a, t_i + b), zphi2s[j]]
        for l in range(1,stop=j)
            t_l = pwl_curve[l][2]
            t_l_1 = pwl_curve[l+1][2]
            push!(conjunctions,
                DisjunctionNode([noIntersection(t_l, t_l_1, t_i, t_i_1 + b), zphi1s[l]])
            )
        end
        push!(disjunctions,ConjunctionNode(conjunctions))
    end
    
    return ConjunctionNode([z_intervalWidth, DisjunctionNode(disjunctions)])
end

"""
mu
Description:
    This function creates a node for an atomic proposition.
    In this work, an atomic proposition holds if it satisfied a linear inequality
    Ax <= b.
Variables:
    segment_index - The index of the curve pwl_curve that we want the atomic proposition
                    mu to hold on.
    pwl_curve - This is an array of arrays defining the curve that the robot
                plans.
    A - Matrix defining the linear inequality.
    b - Matrix/vector defining the right hand side of linear inequality.
"""
function mu(segment_index::Int, pwl_curve, A, b, bloat_factor::Float64=-1.0)
    # 0 is the default bloat_factor
    bloat_factor = max(0,bloat_factor)

    # This segment is fully contained in Ax <= b (shrinked)
    num_edges = length(b)
    conjunctions = []
    for edge_index in range(1,stop=num_edges) # This seems inefficient. Can't we just use Big-M?
        a = A[edge_index,:]
        for j in range(segment_index,stop=segment_index+1)
            x = pwl_curve[j][1]
            push!(conjunctions, b[edge_index] - norm(a) * bloat_factor - sum(a[i] * x[i] for i in range(1,length(a))) - EPS)
        end
    end
    
    return ConjunctionNode(conjunctions)
end

"""
negmu
Description:
    This function creates a node for an atomic proposition.
    In this work, an atomic proposition holds if it satisfied a linear inequality
        Ax \not ≤ b
    
Variables:
    segment_index - The index of the curve pwl_curve that we want the atomic proposition
                    mu to hold on.
    pwl_curve - This is an array of arrays defining the curve that the robot
                plans.
    A - Matrix defining the linear inequality.
    b - Matrix/vector defining the right hand side of linear inequality.
    bloat_factor - Float that defines how much bloating to apply to the linear inequalities.
"""
function negmu(segment_index::Int, pwl_curve, A, b, bloat_factor::Float64=-1.0)::DisjunctionNode
    # 0 is the default bloat_factor
    bloat_factor = max(0,bloat_factor)

    # This segment is fully contained in Ax <= b (shrinked)
    num_edges = length(b)
    disjunctions = []
    for edge_index in range(1,stop=num_edges) # This seems inefficient. Can't we just use Big-M?
        a = A[edge_index,:]
        conjunctions = [] # We need to do conjunction, because we want to make sure that
                          # the FULL line segment x[j] to x[j+1] does not satisfy the task.
        for j in range(segment_index,stop=segment_index+1)
            x = pwl_curve[j][1]
            push!(conjunctions, sum(a[i] * x[i] for i in range(1,length(a))) - b[edge_index] - norm(a) * bloat_factor - EPS)
        end
        
        push!(disjunctions,ConjunctionNode(conjunctions))
    end
    
    return DisjunctionNode(disjunctions)
    
end