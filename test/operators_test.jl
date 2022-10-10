"""
operators_test.jl
Description:
    This function is meant to test the Signal Temporal Logic (STL)
    operators that were defined in the file operators.jl
"""

using Test

include("../src/PWLPlan.jl")

@testset "1. always Tests" begin
    # Create the inputs for always function
    i = 2

    a = 0.5
    b = 2.5

    pwl_curve = Vector{Tuple{Vector{Float64},Float64}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    curve_length = length(pwl_curve)

    z_phis = Vector{Float64}([1.0,1,1,1,1])

    # Call always
    alwaysNode1 = always(i,a,b,z_phis,pwl_curve)
    
    # Test output node
    t_i = pwl_curve[i][2]
    t_i_1 = pwl_curve[i+1][2]
    for piece_index in range(1,stop=curve_length-1)
        t_j = pwl_curve[piece_index][2]
        t_j_1 = pwl_curve[piece_index+1][2]
        @test alwaysNode1.Children[piece_index].Children[1].Children[1] < t_i + a - t_j_1
    end

end

@testset "2. bounded_eventually testing" begin
    # Create two symbolic variables
    pwl_curve = Vector{Tuple{Vector{Any},Any}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    a = 0.5
    b = 2.5

    tmax = 3.7

    node1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],NodeInfo("Simple Node 1"))
    node2 = DisjunctionNode([5.0,-9.1],[],[],NodeInfo("Simple Node 2"))

    # Algorithm 
    t_i = 0.0
    be_node = bounded_eventually(2,a,b,Vector{Float64}([1.0,2.0,3.0,4.0,5.0]),pwl_curve,tmax)
    @test be_node.Children[2] > t_i+b-tmax
end

@testset "3. until testing" begin
    # Create two symbolic variables
    pwl_curve = Vector{Tuple{Vector{Any},Any}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    a = 0.5
    b = 2.5

    node1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],NodeInfo("Simple Node 1"))
    node2 = DisjunctionNode([5.0,-9.1],[],[],NodeInfo("Simple Node 2"))

    # Algorithm
    until_node = until(
        4,
        a,
        b,
        Vector{Float64}([1.0,2.0,3.0,4.0,5.0]),
        Vector{Float64}([1.0,2.0,3.0,4.0,5.0]),
        pwl_curve
    )

    @test until_node.Children[1] < b - a - (pwl_curve[5][2] - pwl_curve[4][2])
end

@testset "4. mu testing" begin
    # Create two symbolic variables
    pwl_curve = Vector{Tuple{Vector{Float64},Float64}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    A1 = [
        1.0 0.0 0.0;
        0.0 1.0 0.0
    ]

    b1 = [
        5.0;
        10.0
    ]

    # Algorithm
    mu_node1 = mu(
        4,
        pwl_curve,
        A1,
        b1
    )
    
    for child in mu_node1.Children
        @test child > 0
    end
end

@testset "5. negmu testing" begin
    # Create two symbolic variables
    pwl_curve = Vector{Tuple{Vector{Float64},Float64}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    A1 = [
        1.0 0.0 0.0;
        0.0 1.0 0.0
    ]

    b1 = [
        1.0;
        2.0
    ]

    # Algorithm
    negmu_node1 = negmu(
        4,
        pwl_curve,
        A1,
        b1
    )
    
    for child in negmu_node1.Children[1].Children
        @test child > 0
    end
end