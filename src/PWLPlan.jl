"""
PWLPlan.jl
Description:
    An implementation of some of the algorithms presented in a file of the same name from STLPlanning.
    Should contain:
        - Implementations of the STLTree node objects.
"""

include("node.jl")
include("plan.jl")
include("operators.jl")

using JuMP, Gurobi

# ==============
# Abstract Types
# ==============


# =========
# Constants
# =========

const EPS = 0.00001
const M = 1e3

# =======================
# Regular Types (Structs)
# =======================

# ==================
# Quick Constructors
# ==================


# =========
# Functions
# =========

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
function plan(x0s::Vector{Vector{Float64}},specifications::Vector{BasicNode},bloat::Float64,limits=[],num_segs::Integer=-1,tasks=[],vmax::Float64=3.0,MIPGap::Float64=1e-4,max_segs::Integer=-1, tmax::Float64=-1.0, hard_goals=[], size=0.11*4/2)
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
        for spec in specifications
            clearSpecTree(spec)
        end

        println("----------------------------")
        println("num_segs = ", num_segs)

        pwl_curves = []
        
        # Create a Gurobi model
        model = Model(Gurobi.Optimizer)
        
        # Ignore some of the parameter setting.

        # Construct constraints on each path (for each robot)
        # in collection.
        for index_a in range(1,stop=length(x0s))
            x0 = x0s[index_a]
            spec = specs[index_a]

            dims = length(x0)

            # Create a curve made up of each of the endpoints of 
            # agent index_a's path.
            pwl_curve = []
            for endpt_indx in range(1,stop=num_segs+1)
                push!(
                    pwl_curve,
                    tuple(
                        @variable(model,[1:dims],base_name = string("x",index_a,"_section",endpt_indx)),
                        @variable(model,base_name = string("t",index_a,"_section",endpt_indx))
                    )
                )
            end
            # This contains num_segs segments, which means we need
            # num_segs + 1 endpoints.
            push!(pwl_curves,pwl_curve)
            
            # Create initial constraints
            @constraint(model, pwl_curve[1][1] .== x0) #Initial state
            @constraint(model, pwl_curve[1][2] == 0.0) #initial time

            # Create goals
            if hard_goals != []
                goal_a = hard_goals[index_a]
                @constraint(model, pwl_curve[length(pwl_curve)][1] .== goal_a )
            end

            # Add Limits on Robots Movement
            add_velocity_constraints(model,Vector{Tuple{Vector{VariableRef},VariableRef}}(pwl_curve),vmax)
            add_time_constraints(model,Vector{Tuple{Vector{VariableRef},VariableRef}}(pwl_curve),tmax)

            handleSpecTree(
                spec,
                Vector{Tuple{Vector{VariableRef},VariableRef}}(pwl_curve),
                bloat,
                size
            )
            add_CDTree_Constraints(model, spec.Zs[1])

        end

        # If there are tasks, then add them to the current agents task tree.
        if length(tasks) > 0
            throw(ErrorException("We are not ready to handle task constraints yet!"))
        end

        # Add Mutual clearance constraints


        # Create objective
        pwl_curve1 = pwl_curves[1]
        obj = pwl_curve1[length(pwl_curve1)][2]
        for curve_index in range(2,stop=length(pwl_curves))
            pwl_curve_i = pwl_curves[curve_index]
            obj += pwl_curve_i[length(pwl_curve_i)][2]
        end
        @objective(model, Min, obj)

        # Try to Report data on the number of variables
        println(string("Num Variables: ",num_variables(model)))

        # Try to optimize
        optimize!(model)

        println(string("Solution time : ", solve_time(model)))

        # IF FEASIBLE
        if objective_value(model) > 0

            # Extract Trajectory
            pwl_curves_output = []
            x_dim = length(x0s[1])
            for curve_index in range(1,stop=length(pwl_curves))
                pwl_i_output = Vector{Tuple{Vector{Float64},Float64}}([])
                for time_stamp_index in range(1,stop=length(pwl_curve1))
                    x_i_k = Vector{Float64}([])
                    for dim_index in range(1,stop=x_dim)
                        push!(x_i_k,value(pwl_curves[curve_index][time_stamp_index][1][dim_index]))
                    end
                    t_i_k = value(pwl_curves[curve_index][time_stamp_index][2])
                    
                    #Add tuple to pwl_i_output
                    push!(
                        pwl_i_output,
                        tuple(x_i_k,t_i_k)
                    )
                end
                push!(pwl_curves_output,pwl_i_output)
            end

            println(pwl_curves_output)
            return pwl_curves_output
        end    

    end
end