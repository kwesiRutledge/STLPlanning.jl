"""
pwlplan_test.jl
Description:
    Testing some of the algorithms developed in the STLPlanning repository.
"""

using Plots, Polyhedra

using Test

include("../src/PWLPlan.jl")

@testset "1. clearSpecTree" begin
    # Create a few sample nodes
    simpleNode1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],"and",NodeInfo("Simple Node 1"))

    @test simpleNode1.Zs != []
    
    clearSpecTree(simpleNode1)
    @test simpleNode1.Zs == []

    # Create a more complicated example

end

@testset "2. plan over 1 step (impossible)" begin
    # Create A problem like run_stlcg-1

    x0 = [  -1. ;
            -1. ]

    wall_half_width = 0.05
    A = [   -1.0 0 ;
            1  0 ;
            0  -1;
            0  1 ]

    walls = []
    push!( walls , [-1.5; -1.5; -1.5; 1.5] )
    push!( walls , [1.5; 1.5; -1.5; 1.5] )
    push!( walls , [-1.5; 1.5; -1.5; -1.5] )
    push!( walls , [-1.5; 1.5; 1.5; 1.5] )

    obs = []
    for wall in walls 
        if (wall[1] == wall[2])
            wall[1] -= wall_half_width
            wall[2] += wall_half_width
        elseif wall[3] == wall[4]
            wall[3] -= wall_half_width
            wall[4] += wall_half_width
        else
            throw(ErrorException("Wrong shape for axis-aligned wall"))
        end
        
        wall = wall .* [-1;1;-1;1]
        push!(obs,tuple(A,wall))
    end

    b1 = [1; -0.7; 0.25; 0.5]
    B1 = tuple(A,b1)

    b2 = [0; 0.9; 1; -0.5]
    B2 = tuple(A,b2)

    b3 = [-0.2; 0.7; -0.8; 1.2]
    B3 = tuple(A,b3)

    c1 = [0.4; 0.4; 0.4; 0.4]
    C1 = tuple(A,c1)

    # Goal
    goal = [1;1]

    # Create constraints
    tmax = 15.0
    vmax = 1.0

    # Create specificationTree
    notC = BasicNode("negmu", NodeInfo(C1[1],C1[2]))
    B1_node = BasicNode("mu", NodeInfo(B1[1],B1[2]))
    B2_node = BasicNode("mu", NodeInfo(B2[1],B2[2]))
    B3_node = BasicNode("mu", NodeInfo(B3[1],B3[2]))

    phi_1 = BasicNode("F", [BasicNode("A", [B2_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
    phi_2 = BasicNode("F", [BasicNode("A", [B3_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
    phi_3 = BasicNode("A", [notC], NodeInfo(0.0,tmax))
    spec = BasicNode("and", [phi_1,phi_2,phi_3], NodeInfo())

    # Call plan
    x0s = [x0]
    specs = [notC]
    goals = [goal]

    try
        pwl_curves = plan(x0s, specs, 0.05 , [], 1, [], vmax, 1e-4, -1, tmax, goals, 0.11*4/2) 
    catch
        # This plan should fail
        @test true
    end

end

@testset "2. plan over 2 steps" begin
    # Create A problem like run_stlcg-1

    x0 = [  -1. ;
            -1. ]

    wall_half_width = 0.05
    A = [   -1.0 0 ;
            1  0 ;
            0  -1;
            0  1 ]

    walls = []
    push!( walls , [-1.5; -1.5; -1.5; 1.5] )
    push!( walls , [1.5; 1.5; -1.5; 1.5] )
    push!( walls , [-1.5; 1.5; -1.5; -1.5] )
    push!( walls , [-1.5; 1.5; 1.5; 1.5] )

    obs = []
    for wall in walls 
        if (wall[1] == wall[2])
            wall[1] -= wall_half_width
            wall[2] += wall_half_width
        elseif wall[3] == wall[4]
            wall[3] -= wall_half_width
            wall[4] += wall_half_width
        else
            throw(ErrorException("Wrong shape for axis-aligned wall"))
        end
        
        wall = wall .* [-1;1;-1;1]
        push!(obs,tuple(A,wall))
    end

    b1 = [1; -0.7; 0.25; 0.5]
    B1 = tuple(A,b1)

    b2 = [0; 0.9; 1; -0.5]
    B2 = tuple(A,b2)

    b3 = [-0.2; 0.7; -0.8; 1.2]
    B3 = tuple(A,b3)

    c1 = [0.4; 0.4; 0.4; 0.4]
    C1 = tuple(A,c1)

    # Goal
    goal = [1;1]

    # Create constraints
    tmax = 15.0
    vmax = 1.0

    # Create specificationTree
    notC = BasicNode("negmu", NodeInfo(C1[1],C1[2]))
    B1_node = BasicNode("mu", NodeInfo(B1[1],B1[2]))
    B2_node = BasicNode("mu", NodeInfo(B2[1],B2[2]))
    B3_node = BasicNode("mu", NodeInfo(B3[1],B3[2]))

    phi_1 = BasicNode("F", [BasicNode("A", [B2_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
    phi_2 = BasicNode("F", [BasicNode("A", [B3_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
    phi_3 = BasicNode("A", [notC], NodeInfo(0.0,tmax))
    spec = BasicNode("and", [phi_1,phi_2,phi_3], NodeInfo())

    # Call plan
    x0s = [x0]
    specs = [phi_3]
    goals = [goal]

    pwl_curves = plan(x0s, specs, 0.05 , [], 2, [], vmax, 1e-4, -1, tmax, goals, 0.11*4/2) 

    # Plot On A Figure
    plot()

    # Plot Regions
    h1 = HalfSpace(A[1,:],b2[1]) ∩ HalfSpace(A[2,:],b2[2]) ∩ HalfSpace(A[3,:],b2[3]) ∩ HalfSpace(A[4,:],b2[4])
    p1 = polyhedron(h1)
    plot!(p1)

    h2 = HalfSpace(A[1,:],b3[1]) ∩ HalfSpace(A[2,:],b3[2]) ∩ HalfSpace(A[3,:],b3[3]) ∩ HalfSpace(A[4,:],b3[4])
    p2 = polyhedron(h2)
    plot!(p2)

    hc = HalfSpace(A[1,:],c1[1]) ∩ HalfSpace(A[2,:],c1[2]) ∩ HalfSpace(A[3,:],c1[3]) ∩ HalfSpace(A[4,:],c1[4])
    pc = polyhedron(hc)
    plot!(pc)

    # Plot Trajectory
    x_traj = zeros((2,length(pwl_curves[1])))
    pwl_curve1 = pwl_curves[1]
    for time_index in range(1,stop=length(pwl_curve1))
        x_t = Vector{Float64}(pwl_curve1[time_index][1])
        x_traj[:,time_index] = x_t 
        println(pwl_curve1[time_index][2])
    end
    plot!(x_traj[1,:],x_traj[2,:])

    savefig("../images/stlcg-test2.png")

    # Test the curve values with the polyhedron for C
    for traj_ind in range(1,stop=size(x_traj,2))
        x_t = x_traj[:,traj_ind]
        println(A * x_t )
        println(A * x_t - c1 .>= 0)
    end

end