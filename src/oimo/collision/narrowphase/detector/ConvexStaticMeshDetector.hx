package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * ConvexGeometry vs StaticMesh collision detector.
 * Handles any convex shape (Sphere, Box, Capsule, ConvexHull, etc.) vs StaticMesh.
 *
 * SIMPLIFIED VERSION - based on working sphere detector approach
 */
@:build(oimo.m.B.bu())
class ConvexStaticMeshDetector extends Detector {

	public function new(swapped:Bool = false) {
		super(swapped);
	}

	override function detectImpl(result:DetectorResult, geom1:Geometry, geom2:Geometry, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var convex:ConvexGeometry = cast geom1;
		var staticMesh:StaticMeshGeometry = cast geom2;

		result.incremental = false;

		// Get a reasonable bounding radius for the convex shape
		// Use a conservative approach based on GJK margin
		var boundingRadius = getBoundingRadius(convex);

		// Transform convex center to mesh local space
		var meshToConvex:IVec3;
		M.vec3_sub(meshToConvex, tf1._position, tf2._position);
		var meshToConvexInMesh:IVec3;
		M.vec3_mulMat3Transposed(meshToConvexInMesh, meshToConvex, tf2._rotation);

		// Convert to Vec3 for geometry queries
		var localConvexCenter = new Vec3();
		M.vec3_toVec3(localConvexCenter, meshToConvexInMesh);

		// Create conservative AABB in mesh local space for broad phase
		var convexAABB = new Aabb();
		M.vec3_set(convexAABB._min, localConvexCenter.x - boundingRadius, localConvexCenter.y - boundingRadius, localConvexCenter.z - boundingRadius);
		M.vec3_set(convexAABB._max, localConvexCenter.x + boundingRadius, localConvexCenter.y + boundingRadius, localConvexCenter.z + boundingRadius);

		// Query potentially colliding triangles
		var triangleIndices = staticMesh.queryTriangles(convexAABB);

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

			// Get the closest point on triangle to convex center (simplified approach)
			var triangleClosestPoint = new Vec3();
			staticMesh.closestPointOnTriangle(localConvexCenter, v1, v2, v3, triangleClosestPoint);

			// Calculate distance from convex center to triangle
			var toConvex = new Vec3().copyFrom(localConvexCenter).subEq(triangleClosestPoint);
			var distance = toConvex.length();

			// Check if this is a collision candidate
			if (distance < boundingRadius && distance < closestDistance) {
				closestDistance = distance;
				closestTriangle = triangleIndex;
				closestPointLocal.copyFrom(triangleClosestPoint);

				// Calculate proper collision normal
				if (distance > 1e-6) {
					// Normal points from triangle towards convex center
					closestNormalLocal.copyFrom(toConvex).scaleEq(1.0 / distance);
				} else {
					// Convex center is on triangle - use face normal
					staticMesh.getTriangleNormal(triangleIndex, closestNormalLocal);
					// Ensure normal points towards convex center
					var testPoint = new Vec3().copyFrom(triangleClosestPoint).addScaledEq(closestNormalLocal, 0.001);
					if (localConvexCenter.sub(testPoint).length() < localConvexCenter.sub(triangleClosestPoint).length()) {
						closestNormalLocal.scaleEq(-1);
					}
				}
			}
		}

		// If we found a collision, create the contact
		if (closestTriangle >= 0) {
			// Calculate actual penetration using support vertex
			var penetrationDepth = calculatePenetrationDepth(convex, tf1, tf2, closestNormalLocal, closestPointLocal, boundingRadius - closestDistance);

			if (penetrationDepth > 0) {
				// Transform collision normal back to world space
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

				// Get contact point on convex using support vertex
				var supportDir = new Vec3().copyFrom(closestNormalLocal).scaleEq(-1);
				var supportPoint = new Vec3();
				convex.computeLocalSupportingVertex(supportDir, supportPoint);

				var contactPointOnConvex:IVec3;
				M.vec3_fromVec3(contactPointOnConvex, supportPoint);
				M.vec3_mulMat3(contactPointOnConvex, contactPointOnConvex, tf1._rotation);
				M.vec3_add(contactPointOnConvex, contactPointOnConvex, tf1._position);

				// Add contact point (this handles the swapped case automatically)
				addPoint(result,
					M.vec3_get(contactPointOnConvex, 0), M.vec3_get(contactPointOnConvex, 1), M.vec3_get(contactPointOnConvex, 2),
					M.vec3_get(contactPointOnMesh, 0), M.vec3_get(contactPointOnMesh, 1), M.vec3_get(contactPointOnMesh, 2),
					penetrationDepth, 0);
			}
		}
	}

	private function getBoundingRadius(convex:ConvexGeometry):Float {
		// Calculate accurate bounding radius for each shape type
		switch (convex._type) {
			case GeometryType._SPHERE:
				var sphere:SphereGeometry = cast convex;
				return sphere.getRadius();
			case GeometryType._BOX:
				// Accurate bounding radius: distance from center to corner
				var box:BoxGeometry = cast convex;
				var halfExtents = box.getHalfExtents();
				var hw = halfExtents.x;
				var hh = halfExtents.y;
				var hd = halfExtents.z;
				return Math.sqrt(hw * hw + hh * hh + hd * hd);
			case GeometryType._CAPSULE:
				// Accurate bounding radius: half-length plus radius
				var capsule:CapsuleGeometry = cast convex;
				return capsule.getHalfHeight() + capsule.getRadius();
			case GeometryType._CYLINDER:
				// Accurate bounding radius for cylinder
				var cylinder:CylinderGeometry = cast convex;
				var hr = cylinder.getRadius();
				var hh = cylinder.getHalfHeight();
				return Math.sqrt(hr * hr + hh * hh);
			case GeometryType._CONE:
				// Accurate bounding radius for cone
				var cone:ConeGeometry = cast convex;
				var hr = cone.getRadius();
				var hh = cone.getHalfHeight();
				return Math.sqrt(hr * hr + hh * hh);
			default:
				// For convex hulls and unknown shapes, compute actual bounding radius
				return computeBoundingRadius(convex);
		}
	}

	private function calculatePenetrationDepth(convex:ConvexGeometry, tf1:Transform, tf2:Transform, normal:Vec3, closestPoint:Vec3, estimatedDepth:Float):Float {
		// Use support vertices for more accurate penetration depth calculation
		switch (convex._type) {
			case GeometryType._SPHERE:
				// For spheres, the estimated depth is already accurate
				return Math.max(0, estimatedDepth);

			default:
				// For other convex shapes, use support function for better accuracy
				var supportDir = new Vec3().copyFrom(normal).scaleEq(-1); // Direction towards mesh
				var supportPoint = new Vec3();
				convex.computeLocalSupportingVertex(supportDir, supportPoint);

				// Transform support point to mesh local space
				var supportWorldPos:IVec3;
				M.vec3_fromVec3(supportWorldPos, supportPoint);
				M.vec3_mulMat3(supportWorldPos, supportWorldPos, tf1._rotation);
				M.vec3_add(supportWorldPos, supportWorldPos, tf1._position);

				// Convert to mesh local space
				var supportInMesh:IVec3;
				M.vec3_sub(supportInMesh, supportWorldPos, tf2._position);
				M.vec3_mulMat3Transposed(supportInMesh, supportInMesh, tf2._rotation);

				var supportInMeshVec3 = new Vec3();
				M.vec3_toVec3(supportInMeshVec3, supportInMesh);

				// Calculate distance from support point to triangle in normal direction
				// This gives us a more accurate penetration depth
				var supportToTriangleCenter = supportInMeshVec3.sub(closestPoint);
				var projectedDistance = supportToTriangleCenter.dot(normal);

				// Return the positive penetration depth
				return Math.max(0, -projectedDistance);
		}
	}

	private function computeBoundingRadius(convex:ConvexGeometry):Float {
		// For unknown convex shapes, compute the actual bounding radius by sampling
		// support vertices in multiple directions to find the maximum distance from center
		var maxRadius:Float = 0.0;
		var supportPoint = new Vec3();

		// Sample directions: axis-aligned directions
		var directions = [
			new Vec3(1, 0, 0), new Vec3(-1, 0, 0),    // +X, -X
			new Vec3(0, 1, 0), new Vec3(0, -1, 0),    // +Y, -Y
			new Vec3(0, 0, 1), new Vec3(0, 0, -1),    // +Z, -Z
		];

		// Also sample some diagonal directions for better coverage
		var sqrt3inv = 1.0 / Math.sqrt(3.0);
		var sqrt2inv = 1.0 / Math.sqrt(2.0);
		directions.push(new Vec3(sqrt3inv, sqrt3inv, sqrt3inv));   // +X+Y+Z
		directions.push(new Vec3(-sqrt3inv, -sqrt3inv, -sqrt3inv)); // -X-Y-Z
		directions.push(new Vec3(sqrt3inv, sqrt3inv, -sqrt3inv));   // +X+Y-Z
		directions.push(new Vec3(sqrt3inv, -sqrt3inv, sqrt3inv));   // +X-Y+Z
		directions.push(new Vec3(-sqrt3inv, sqrt3inv, sqrt3inv));   // -X+Y+Z
		directions.push(new Vec3(sqrt2inv, sqrt2inv, 0));          // +X+Y
		directions.push(new Vec3(sqrt2inv, -sqrt2inv, 0));         // +X-Y
		directions.push(new Vec3(sqrt2inv, 0, sqrt2inv));          // +X+Z
		directions.push(new Vec3(sqrt2inv, 0, -sqrt2inv));         // +X-Z
		directions.push(new Vec3(0, sqrt2inv, sqrt2inv));          // +Y+Z
		directions.push(new Vec3(0, sqrt2inv, -sqrt2inv));         // +Y-Z

		// Find the support vertex in each direction and compute distance from origin
		for (dir in directions) {
			convex.computeLocalSupportingVertex(dir, supportPoint);
			var distance = supportPoint.length();
			if (distance > maxRadius) {
				maxRadius = distance;
			}
		}

		// Add a small margin for safety (in case our sampling missed the true maximum)
		return maxRadius + convex._gjkMargin;
	}
}
