package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Sphere vs StaticMesh collision detector.
 */
@:build(oimo.m.B.bu())
class SphereStaticMeshDetector extends Detector {

	public function new(swapped:Bool = false) {
		super(swapped);
	}

	override function detectImpl(result:DetectorResult, geom1:Geometry, geom2:Geometry, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var sphere:SphereGeometry = cast geom1;
		var staticMesh:StaticMeshGeometry = cast geom2;

		result.incremental = false;

		var sphereRadius:Float = sphere.getRadius();

		// Transform sphere center to mesh local space
		var meshToSphere:IVec3;
		M.vec3_sub(meshToSphere, tf1._position, tf2._position);
		var meshToSphereInMesh:IVec3;
		M.vec3_mulMat3Transposed(meshToSphereInMesh, meshToSphere, tf2._rotation);

		// Convert to Vec3 for geometry queries
		var localSphereCenter = new Vec3();
		M.vec3_toVec3(localSphereCenter, meshToSphereInMesh);

		// Create sphere AABB in mesh local space for broad phase
		var sphereAABB = new Aabb();
		M.vec3_set(sphereAABB._min, localSphereCenter.x - sphereRadius, localSphereCenter.y - sphereRadius, localSphereCenter.z - sphereRadius);
		M.vec3_set(sphereAABB._max, localSphereCenter.x + sphereRadius, localSphereCenter.y + sphereRadius, localSphereCenter.z + sphereRadius);

		// Query potentially colliding triangles
		var triangleIndices = staticMesh.queryTriangles(sphereAABB);

		var closestDistance = MathUtil.POSITIVE_INFINITY;
		var closestTriangle = -1;
		var closestPointLocal = new Vec3();
		var closestNormalLocal = new Vec3();

		// Test each potentially colliding triangle
		for (triangleIndex in triangleIndices) {
			var v1 = new Vec3();
			var v2 = new Vec3();
			var v3 = new Vec3();
			staticMesh.getTriangleVertices(triangleIndex, v1, v2, v3);

			var triangleClosestPoint = new Vec3();
			staticMesh.closestPointOnTriangle(localSphereCenter, v1, v2, v3, triangleClosestPoint);

			// Calculate distance from sphere center to triangle
			var toSphere = new Vec3().copyFrom(localSphereCenter).subEq(triangleClosestPoint);
			var distance = toSphere.length();

			// Check if this is a collision and closer than previous ones
			if (distance <= sphereRadius && distance < closestDistance) {
				closestDistance = distance;
				closestTriangle = triangleIndex;
				closestPointLocal.copyFrom(triangleClosestPoint);

				// Calculate proper collision normal
				if (distance > 1e-6) {
					// Normal points from triangle towards sphere center
					closestNormalLocal.copyFrom(toSphere).scaleEq(1.0 / distance);
				} else {
					// Sphere center is on triangle - use face normal
					staticMesh.getTriangleNormal(triangleIndex, closestNormalLocal);
					// Ensure normal points towards sphere center
					var testPoint = new Vec3().copyFrom(triangleClosestPoint).addScaledEq(closestNormalLocal, 0.001);
					if (localSphereCenter.sub(testPoint).length() < localSphereCenter.sub(triangleClosestPoint).length()) {
						closestNormalLocal.scaleEq(-1);
					}
				}
			}
		}

		// If we found a collision, create the contact
		if (closestTriangle >= 0) {
			var penetrationDepth = sphereRadius - closestDistance;

			// Transform collision normal back to world space using proper IVec3 operations
			var normalInWorld:IVec3;
			M.vec3_fromVec3(normalInWorld, closestNormalLocal);
			M.vec3_mulMat3(normalInWorld, normalInWorld, tf2._rotation);

			// Set collision normal (this handles the swapped case automatically)
			setNormal(result, M.vec3_get(normalInWorld, 0), M.vec3_get(normalInWorld, 1), M.vec3_get(normalInWorld, 2));

			// Calculate contact points in world space
			var contactPointOnMesh:IVec3;
			M.vec3_fromVec3(contactPointOnMesh, closestPointLocal);
			M.vec3_mulMat3(contactPointOnMesh, contactPointOnMesh, tf2._rotation);
			M.vec3_add(contactPointOnMesh, contactPointOnMesh, tf2._position);

			var contactPointOnSphere:IVec3;
			M.vec3_addRhsScaled(contactPointOnSphere, tf1._position, normalInWorld, -sphereRadius);

			// Add contact point (this handles the swapped case automatically)
			addPoint(result,
				M.vec3_get(contactPointOnSphere, 0), M.vec3_get(contactPointOnSphere, 1), M.vec3_get(contactPointOnSphere, 2),
				M.vec3_get(contactPointOnMesh, 0), M.vec3_get(contactPointOnMesh, 1), M.vec3_get(contactPointOnMesh, 2),
				penetrationDepth, 0);
		}
	}
}
