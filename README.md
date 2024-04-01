# OrbitalTrajectory
A project for finding the optimal thrust control and orbital trajectory to orbit a rocket around the moon


## Equation Derivation

Here is a detailed derivartion of the equations used in this project. Every symbol comes from the following figure:


<img title="Figure 1" alt="Main figure for this project" src="figures/crazy_figure.png">


 These equations will also be in the $\LaTeX$ write-up. We will first derive $\sin(\psi)$ and $\cos(\psi)$ in terms of $i$ and $x$, and then further derive what those equations are. We first derive $\sin(\psi)$:

$$\begin{aligned}
    \sin(\psi) &= \sin(90+i - x) \\
           &= \sin(90+i)\cos(x) - \cos(90+i)\sin(x) \\\
           &= \big(\sin(90)\cos(i) + \cos(90)\sin(i) \big)\cos(x) \\
           &= -\big(\cos(90)\cos(i) - \sin(90)\sin(i) \big)\sin(x) \\
           &= \cos(i)\cos(x) + \sin(i)\sin(x). \\
\end{aligned}$$

We now derive $\cos(\psi)$:

$$\begin{aligned}
    \cos(\psi) &= \cos(90+i - x) \\
               &= \cos(90+i)\cos(x) + \sin(90+i)\sin(x) \\
               &= \big(\cos(90)\cos(i) - \sin(90)\sin(i) \big)\cos(x) + \big(\sin(90)\cos(i) + \cos(90)\sin(i) \big)\sin(x) \\
               &= \cos(i)\sin(x) - \sin(i)\cos(x). \\
\end{aligned}$$

We $\sin(\psi)$ and $\cos(\psi)$ defined, we now derive $\sin$ and $\cos$ in terms of $i$ and $x$.

First, note that 

$$\begin{equation*}
    \frac{\sin(i)}{E} = \frac{\sin(\theta)}{J}
\end{equation*}$$

where 

$$\begin{aligned}
    E &= d\cos(\theta) \\
    J &= d\sin(\theta) - E\tan(\phi) = d\sin(\theta) - d\cos(\theta)\tan(\phi).
\end{aligned}$$

Substituting $E$ and $J$ into the equation above, we get

$$\begin{aligned}
    \frac{\sin(i)}{d\cos(\theta)} &= \frac{\sin(\theta)}{d\sin(\theta) - d\cos(\theta)\tan(\phi)} \\
    \sin(i) &= \frac{d\cos(\theta)\sin(\theta)}{d\sin(\theta) - d\cos(\theta)\tan(\phi)} \\
\end{aligned}$$

Thus we have $\sin(i)$. Note, for $\sin(x)$, we have

$$\begin{equation*}
    \frac{\sin(x)}{R} = \frac{\sin(\theta-\phi)}{d_m}.
\end{equation*}$$

Thus,

$$\begin{equation*}
    \sin(x) = \frac{R\sin(\theta-\phi)}{d_m}.
\end{equation*}$$

Now that we have $\sin(i)$ and $\sin(x)$, we now turn our attention to $\cos(i)$ and $\cos(x)$. First, note that

$$\begin{equation*}
    \cos(A) = \frac{a^2 - b^2 - c^2}{-2bc} = \frac{b^2 + c^2 - a^2}{2bc}.
\end{equation*}$$

Thus in our situation, we have

$$\begin{equation*}
    \cos(x) = \frac{d^2 + d_m^2 - R^2}{2dd_m}.
\end{equation*}$$

We now define $\cos(i)$ to be

$$\begin{equation*}
    \cos(i) = \frac{d^2 + J^2 - G^2}{2dJ}.
\end{equation*}$$

Recall previously that $J = d\sin(\theta) - d\cos(\theta)\tan(\phi)$. Since $\cos(\phi) = \frac{E}{G} \implies G = \frac{E}{\cos(\phi)}$, we have $G = \frac{d\cos(\theta)}{\cos(\phi)}$. Plugging these values of $J$ and $G$ into the equation for $\cos(i)$, we get

$$\begin{equation*}
    \cos(i) = \frac{d^2 + J^2 - G^2}{2dJ} = \frac{d^2 + (d\sin(\theta) - d\cos(\theta)\tan(\phi))^2 - \left(\frac{d\cos(\theta)}{\cos(\phi)}\right)^2}{2d\big(d\sin(\theta) - d\cos(\theta)\tan(\phi)\big)}.
\end{equation*}$$

We now have the appropriate derivations for $\sin(i)$, $\sin(x)$, $\cos(i)$, and $\cos(x)$. To summarize, we have

$$\begin{aligned}
    \sin(i) &= \frac{d\cos(\theta)\sin(\theta)}{d\sin(\theta) - d\cos(\theta)\tan(\phi)}  &\sin(x) = \frac{R\sin(\theta-\phi)}{d_m} \\
    \cos(i) &= \frac{d^2 + (d\sin(\theta) - d\cos(\theta)\tan(\phi))^2 - \left(\frac{d\cos(\theta)}{\cos(\phi)}\right)^2}{2d\big(d\sin(\theta) - d\cos(\theta)\tan(\phi)\big)} &\cos(x) = \frac{d^2 + d_m^2 - R^2}{2dd_m}.
\end{aligned}$$


