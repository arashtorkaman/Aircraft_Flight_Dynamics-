# GitHub Math Rendering Test

This file uses the syntax documented by GitHub for mathematical expressions in Markdown.

## Inline equation

The longitudinal aircraft model is $\dot{x}=Ax+Bu$.

## Display equation

$$\dot{x}=Ax+Bu$$

## Matrix

$$x=\begin{bmatrix}u&w&q&\theta\end{bmatrix}^{T}$$

## Closed-loop model

$$A_{cl}=A-BK$$

## LQR cost function

$$J=\int_{0}^{\infty}\left(x^{T}Qx+u^{T}Ru\right)\,dt$$

## Riccati equation

$$A^{T}P+PA-PBR^{-1}B^{T}P+Q=0$$

If these equations render on the GitHub website but not in your local editor, the local Markdown preview does not have GitHub MathJax support enabled.
