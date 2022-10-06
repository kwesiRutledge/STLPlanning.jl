"""
plan_test.jl
Description:
    Making use of the 
"""

using Test
include("../src/PWLPlan.jl")

@testset "1. plan test" begin

    # Constants
    x0s=[Vector([1.0,2.0])]
    
    root = DisjunctionNode([1.0])
    specs = [root,]

    limits=[]
    tasks=[]
    vmax::Float64=3.0
    MIPGap::Float64=1e-4
    max_segs::Integer=-1
    tmax::Float64=-1.0
    hard_goals=[]
    size=0.11*4/2
    num_segs = 10
    
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

    @test min_segs == 10
    @test max_segs == 10

    # Iterate through the many potential number of segments
    for num_segs in range(min_segs,stop=max_segs)
        for spec in specs
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
            pwl_curve = Vector{Vector{Any}}([])
            for i in range(1,stop=num_segs+1)
                push!(
                    pwl_curve,
                    [@variable(
                        model,
                        [1:dims],
                        base_name = string("x",index_a,"_section",i,"_")
                        ),
                    @variable(
                        model,
                        base_name=string("t",index_a,"_section",i,"_")
                        )
                    ]
                )
            end
            # println(pwl_curve)
            # println(pwl_curve[1])

            # This contains num_segs segments, which means we need
            # num_segs + 1 endpoints.
            push!(pwl_curves,pwl_curve)
            
            # Create initial constraints
            for dim_index in range(1,stop=dims)
                @constraint(model, pwl_curve[1][1][dim_index,1] == x0[dim_index])
            end
            @constraint(model, pwl_curve[1][2] == 0)

            # Goal constraints
            if hard_goals != []
                goal = hard_goals[index_a]
                for dim_index in range(1,stop=dims)
                    @constraint(model,pwl_curve[num_segs][1][dim_index] == goal[dim_index])
                end
            end

            # Limits Constraints
            if limits != []
                add_space_constraints(model,[P[0] for P in pwl_curve ],limits)
            end
            
        end
    end

end

@testset "2. add_space_constraints (Using Two Dimensions)" begin
    # Create a simple JuMP Model and limits for the model
    model = Model(Gurobi.Optimizer)

    pwl_curve = []
    for endpt_indx in range(1,stop=11)
        push!(
            pwl_curve,
            tuple(@variable(model,[1:2],base_name = string("x_section",endpt_indx,"_")),@variable(model,base_name = string("t_section",endpt_indx,"_")) )
            )
    end

    bounds = [1.0,3.0,5.0,7.0]

    add_space_constraints(model,[P[1] for P in pwl_curve ],bounds)

    optimize!(model)
    # @show value(pwl_curve[1][1][1])
    
    # Get the value of every end point.
    for endpt_indx in range(1,stop=length(pwl_curve))
        tuple0 = pwl_curve[endpt_indx]
        pt0 = [value(tuple0[1][1]); value(tuple0[1][2])]

        @test bounds[ [1,3] ] <= pt0
        @test pt0 <= bounds[ [2,4] ]
    end
end

@testset "3. add_velocity_constraints" begin
    # Create a simple JuMP Model and limits for the model
    model = Model(Gurobi.Optimizer)

    pwl_curve = Vector{Tuple{Vector{VariableRef},VariableRef}}([])
    for endpt_indx in range(1,stop=11)
        push!(
            pwl_curve,
            tuple(@variable(model,[1:2],base_name = string("x_section",endpt_indx,"_")),@variable(model,base_name = string("t_section",endpt_indx,"_")) )
            )
    end

    vmax = 4.0

    add_velocity_constraints(model, pwl_curve , vmax)

    optimize!(model)

    # Get the value of every end point.
    for endpt_indx in range(1,stop=length(pwl_curve)-1)
        tuple0 = pwl_curve[endpt_indx]
        x_i = [value(tuple0[1][1]); value(tuple0[1][2])]
        t_i = value(tuple0[2])

        tuple1 = pwl_curve[endpt_indx+1]
        x_ip1 = [value(tuple1[1][1]); value(tuple1[1][2])]
        t_ip1 = value(tuple1[2])

        @test norm((x_ip1-x_i)/(t_ip1-t_i),1) < vmax
    end

end