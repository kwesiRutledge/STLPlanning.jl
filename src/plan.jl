"""
plan.jl
Description:
    Contains the plan() functions that I will use to do planning for STL.
"""

using JuMP, Gurobi

#############
# Constants #
#############

const T_MIN_SEP = 1e-2

####################
# Helper Functions #
####################

"""
add_space_constraints
Description:

Inputs:
    jump_model: A model created for JuMP optimization.
    curve_endpoints: A Vector containing states of the curve (these should be Vectors containing
                    optimization variables)
    bounds: a 2n length vector where n is the dimension of the state space.
Example Usage:

"""
function add_space_constraints(jump_model,curve_endpoints::Vector{Vector{VariableRef}},bounds::Vector{Float64})
    # Constants
    dim = length(curve_endpoints[1])
    
    # Check to see that there are 4 elements in bounds array
    if length(bounds) != 4
        throw(ErrorException("The bounds vector should only have 4 elements!"))
    end

    # Create bounds as constraints on a JuMP model
    for x in curve_endpoints
        for dim_index in range(1,stop=dim)
            @constraint(jump_model, bounds[1+2*(dim_index-1)] .<= x[dim_index] .<= bounds[2+2*(dim_index-1)])
        end
    end
end

"""
add_velocity_constraints(model,pwl_curve,vmax)
Description:

Inputs:

Example Usage:
    add_velocity_constraints(model,pwl_curve,3.5)

"""
function add_velocity_constraints(jump_model,pwl_curve::Vector{Tuple{Vector{VariableRef},VariableRef}},vmax::Float64=3.0)
    # Constants
    dim = length(pwl_curve[1][1])

    # Iterate through each segment
    for segment_index in range(1,stop=length(pwl_curve)-1)
        x1, t1 = pwl_curve[segment_index]
        x2, t2 = pwl_curve[segment_index+1]

        @constraint(jump_model,t1 <= t2)

        # create L1 distance expression?
        for l1_norm_constraint_index in range(0,stop=2^dim-1 )
            # Create vector a, which we will mulyiply x to create constraints for ||x||_1 (a*x)
            a = digits(l1_norm_constraint_index, base=2, pad=dim) -1 * (ones(dim,1) - digits(l1_norm_constraint_index, base=2, pad=dim) )
            @constraint(jump_model,  (transpose(a) * (x2 - x1)) .≤ vmax * (t2 - t1) )
            @constraint(jump_model,  -vmax * (t2 - t1) .≤ (transpose(a) * (x2 - x1)))
        end
    end
end

"""
add_time_constraints(model,pwl_curve,tmax)
Description:
    Adds a constraint on how each segment's start time, so that step t_i
    always begins before step t_{i+1}.
Inputs:

Example Usage:
    add_time_constraints(model,pwl_curve,3.5)

"""
function add_time_constraints(jump_model,pwl_curve::Vector{Tuple{Vector{VariableRef},VariableRef}},tmax::Float64=-1.0)
    # Constants
    if tmax < 0
        @constraint(jump_model,pwl_curve[length(pwl_curve)][2] <= tmax - T_MIN_SEP)
    end

    # Iterate through each segment
    for segment_index in range(1,stop=length(pwl_curve)-1)
        x1, t1 = pwl_curve[segment_index]
        x2, t2 = pwl_curve[segment_index+1]

        @constraint(jump_model, t2 - t1 >= T_MIN_SEP )
    end
end