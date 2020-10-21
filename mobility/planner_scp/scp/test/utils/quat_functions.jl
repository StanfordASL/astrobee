### CONVENTIONS ###
# quaternion: q = [rho q4] where rho is unit vector

# References
# A Survey of Attitude Representations; Shuster 1993
# A General Construction Scheme for Unit Quaternion Curves with Simple High Order Derivatives; Kim et al. 1995

function skew{T}(v::Vector{T})
  # generates 3x3 skew symmetric matrix for vector
  return [0     -v[3]   v[2];
          v[3]   0      -v[1];
         -v[2]   v[1]   0]
end

function quat_derivative{T}(q::Vector{T}, w::Vector{T})
  # Eq. 305 in Shuster
  omega_skew = [-skew(w) w; -w' 0]
  return 0.5*omega_skew*q
end

function quat2angle{T}(q::Vector{T})
  return 2*atan2(norm(q[1:3]), q[4])
end


function quat2vec{T}(q::Vector{T}, canonicalize::Bool=false)
  # canonicalize: true corresponds to positive scalar component q for dual representation
  if norm(q[1:3])^2 < 0.00001
    return zeros(T,3)
  end

  a = canonicalize ? sign(q[4])*q[1:3]/norm(q[1:3]) : q[1:3]/norm(q[1:3])
  return a
end

function quat_multiply{T}(q1::Vector{T}, q2::Vector{T})
  # Eq. 169-171 in Shuster
  # q1 * q2 <==> R(q1) * R(q2)

  q_product = zeros(T,4)
  q_product[1:3] = q1[4]*q2[1:3] + q2[4]*q1[1:3] - cross(Vector(q1[1:3]), Vector(q2[1:3]))
  q_product[4] = q1[4]*q2[4] - dot(q1[1:3], q2[1:3])
  return q_product
end

function quat_inv{T}(q::Vector{T})
  # Eq. 177 in Shuster
  qinv = copy(q)
  qinv[1:3]*=-1
  return qinv
end

function quat_exp{T}(v::Vector{T})
  # gives quaternion q corresponding to orientation obtained
  # from an initial orientation [0 0 0 1] by rotating of an angle norm(v)
  # around fixed direction v/norm(v)
  # Eq. 9 in "Time-Optimal Reorientation of a Spacecraft Using an Inverse Dynamics Optimization Method"
  phi = norm(v)
  if phi^2 < 0.00001
    return Vector{T}([0;0;0;1])
  end
  return Vector{T}([sin(0.5*phi)*v/phi;cos(0.5*phi)])
end

function quat_log{T}(q::Vector{T})
  # gives vector having direction of the Euler's axis and magnitude equal
  # to Euler's angle of the orientation from [0 0 0 1] to q
  # Eq. 9 in "Time-Optimal Reorientation of a Spacecraft Using an Inverse Dynamics Optimization Method"

  if norm(q[1:3])^2 < 0.00001
    return zeros(T,3)
  end
  return quat2angle(q)*quat2vec(q)
end

function quat_interp{T}(q0::Vector{T}, q1::Vector{T}, t::T)
  # p371 in "A General Construction Scheme for Unit Quaternion Curves with Simple High Order Derivatives"
  # NOTE: q1 * q2 <==> R(q1) * R(q2)

  if (t>1 || t<0)
    print("Supplied time t must lie in [0,1]!\n")
    return t<0 ? q0 : q1
  end
  
  qerr = quat_multiply(quat_inv(q0),q1)
  return quat_multiply(q0, quat_exp(t*quat_log(qerr)))
end
