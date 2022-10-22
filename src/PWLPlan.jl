"""
PWLPlan.jl
Description:
    An implementation of some of the algorithms presented in a file of the same name from STLPlanning.
    Should contain:
        - Implementations of the STLTree node objects.
"""
 
# Required Modules
using JuMP, Gurobi

# # Export STLPlanning Types
# export Node
# export BasicNode, DisjunctionNode, ConjunctionNode
# export NodeInfo

# Source Code
include("node.jl")
include("operators.jl")
include("plan.jl")

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