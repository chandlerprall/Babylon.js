var BABYLON;
(function (BABYLON) {
    var GoblinPlugin = (function () {
        function GoblinPlugin() {
            this._registeredMeshes = [];
        }
        GoblinPlugin.prototype.initialize = function (iterations) {
            if (typeof iterations === "undefined") { iterations = 10; }
            this._world = new Goblin.World(new Goblin.BasicBroadphase(), new Goblin.NarrowPhase(), new Goblin.IterativeSolver());
            this._world.solver.max_iterations = iterations;
        };

        GoblinPlugin.prototype.runOneStep = function (delta) {
            this._world.step(delta);

            for (var index = 0; index < this._registeredMeshes.length; index++) {
                var registeredMesh = this._registeredMeshes[index];

                if (registeredMesh.isChild) {
                    continue;
                }

                // Body position
                var bodyX = registeredMesh.body.position.x, bodyY = registeredMesh.body.position.y, bodyZ = registeredMesh.body.position.z;

                var deltaPos = registeredMesh.delta;
                if (deltaPos) {
                    registeredMesh.mesh.position.x = bodyX + deltaPos.x;
                    registeredMesh.mesh.position.y = bodyZ + deltaPos.y;
                    registeredMesh.mesh.position.z = bodyY + deltaPos.z;
                } else {
                    registeredMesh.mesh.position.x = bodyX;
                    registeredMesh.mesh.position.y = bodyZ;
                    registeredMesh.mesh.position.z = bodyY;
                }

                if (!registeredMesh.mesh.rotationQuaternion) {
                    registeredMesh.mesh.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
                }

                registeredMesh.mesh.rotationQuaternion.x = registeredMesh.body.rotation.x;
                registeredMesh.mesh.rotationQuaternion.y = registeredMesh.body.rotation.z;
                registeredMesh.mesh.rotationQuaternion.z = registeredMesh.body.rotation.y;
                registeredMesh.mesh.rotationQuaternion.w = -registeredMesh.body.rotation.w;
            }
        };

        GoblinPlugin.prototype.setGravity = function (gravity) {
            this._world.gravity.set(gravity.x, gravity.z, gravity.y);
        };

        GoblinPlugin.prototype.registerMesh = function (mesh, impostor, options) {
            this.unregisterMesh(mesh);

            mesh.computeWorldMatrix(true);

            var body;
            switch (impostor) {
                case BABYLON.PhysicsEngine.SphereImpostor:
                    var bbox = mesh.getBoundingInfo().boundingBox;
                    var radiusX = bbox.maximumWorld.x - bbox.minimumWorld.x;
                    var radiusY = bbox.maximumWorld.y - bbox.minimumWorld.y;
                    var radiusZ = bbox.maximumWorld.z - bbox.minimumWorld.z;

                    body = new Goblin.RigidBody(new Goblin.SphereShape(Math.max(radiusX, radiusY, radiusZ) / 2), options.mass);
                    break;
                case BABYLON.PhysicsEngine.BoxImpostor:
                    bbox = mesh.getBoundingInfo().boundingBox;
                    var min = bbox.minimumWorld;
                    var max = bbox.maximumWorld;
                    var box = max.subtract(min).scale(0.5);

                    body = new Goblin.RigidBody(new Goblin.BoxShape(box.x, box.y, box.z), options.mass);
                    break;
                case BABYLON.PhysicsEngine.PlaneImpostor:
                    bbox = mesh.getBoundingInfo().boundingBox;
                    var min = bbox.minimumWorld;
                    var max = bbox.maximumWorld;
                    var box = max.subtract(min).scale(0.5);

                    body = new Goblin.RigidBody(new Goblin.PlaneShape(1, box.x, box.z), options.mass);
                    break;
                case BABYLON.PhysicsEngine.MeshImpostor:
                    var rawVerts = mesh.getVerticesData(BABYLON.VertexBuffer.PositionKind), verts = [];
                    for (var i = 0; i < rawVerts.length; i += 3) {
                        verts.push(new Goblin.Vector3(rawVerts[i], rawVerts[i + 1], rawVerts[i + 2]));
                    }

                    body = new Goblin.RigidBody(new Goblin.MeshShape(verts, mesh.getIndices()), options.mass);
                    break;
            }

            body.friction = options.friction;
            body.restitution = options.restitution;

            var initialRotation = null;

            if (mesh.rotationQuaternion) {
                initialRotation = mesh.rotationQuaternion.clone();
                mesh.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
            } else {
                initialRotation = BABYLON.Quaternion.RotationYawPitchRoll(mesh.rotation.y, mesh.rotation.x, mesh.rotation.z);
            }

            // The delta between the mesh position and the mesh bounding box center
            var bbox = mesh.getBoundingInfo().boundingBox;
            var deltaPosition = mesh.position.subtract(bbox.center);

            if (initialRotation) {
                body.rotation.x = initialRotation.x;
                body.rotation.z = initialRotation.y;
                body.rotation.y = initialRotation.z;
                body.rotation.w = -initialRotation.w;
            }

            body.position.set(bbox.center.x, bbox.center.z, bbox.center.y);
            this._world.addRigidBody(body);

            this._registeredMeshes.push({ mesh: mesh, body: body, delta: deltaPosition });

            return body;
        };

        GoblinPlugin.prototype.registerMeshesAsCompound = function (parts, options) {
            var compoundShape = new Goblin.CompoundShape();

            for (var index = 0; index < parts.length; index++) {
                var mesh = parts[index].mesh;

                var shape = this.registerMesh(mesh, parts[index].impostor);

                if (index == 0) {
                    compoundShape.addChildShape(shape, new Goblin.Vector3(0, 0, 0), new Goblin.Quaternion(0, 0, 0, 1));
                } else {
                    compoundShape.addChildShape(shape, new Goblin.Vector3(mesh.position.x, mesh.position.z, mesh.position.y), new Goblin.Quaternion(mesh.rotationQuaternion.x, mesh.rotationQuaternion.y, mesh.rotationQuaternion.z, mesh.rotationQuaternion.w));
                }
            }

            var body = new Goblin.RigidBody(shape, options.mass);
            body.friction = options.friction;
            body.restitution = options.restitution;

            var initialRotation = null;

            if (mesh.rotationQuaternion) {
                initialRotation = mesh.rotationQuaternion.clone();
                mesh.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
            }

            // The delta between the mesh position and the mesh bounding box center
            var bbox = mesh.getBoundingInfo().boundingBox;
            var deltaPosition = mesh.position.subtract(bbox.center);

            if (initialRotation) {
                body.rotation.x = initialRotation.x;
                body.rotation.z = initialRotation.y;
                body.rotation.y = initialRotation.z;
                body.rotation.w = -initialRotation.w;
            }

            body.position.set(bbox.center.x, bbox.center.z, bbox.center.y);
            this._world.addRigidBody(body);

            this._registeredMeshes.push({ mesh: mesh, body: body, delta: deltaPosition });

            return body;
        };

        GoblinPlugin.prototype.unregisterMesh = function (mesh) {
            for (var index = 0; index < this._registeredMeshes.length; index++) {
                var registeredMesh = this._registeredMeshes[index];

                if (registeredMesh.mesh === mesh) {
                    // Remove body
                    if (registeredMesh.body) {
                        this._world.removeRigidBody(registeredMesh.body);
                    }

                    this._registeredMeshes.splice(index, 1);
                    return;
                }
            }
        };

        GoblinPlugin.prototype.applyImpulse = function (mesh, force, contactPoint) {
            var worldPoint = new Goblin.Vector3(contactPoint.x, contactPoint.z, contactPoint.y);
            var impulse = new Goblin.Vector3(force.x, force.z, force.y);

            for (var index = 0; index < this._registeredMeshes.length; index++) {
                var registeredMesh = this._registeredMeshes[index];

                if (registeredMesh.mesh === mesh) {
                    impulse.scale(registeredMesh.body.mass);
                    registeredMesh.body.applyForceAtWorldPoint(impulse, worldPoint);
                    return;
                }
            }
        };

        GoblinPlugin.prototype.createLink = function (mesh1, mesh2, pivot1, pivot2) {
            /*var body1 = null, body2 = null;
            for (var index = 0; index < this._registeredMeshes.length; index++) {
            var registeredMesh = this._registeredMeshes[index];
            
            if (registeredMesh.mesh === mesh1) {
            body1 = registeredMesh.body;
            } else if (registeredMesh.mesh === mesh2) {
            body2 = registeredMesh.body;
            }
            }
            
            if (!body1 || !body2) {
            return false;
            }
            
            var constraint = new CANNON.PointToPointConstraint(body1, new CANNON.Vec3(pivot1.x, pivot1.z, pivot1.y), body2, new CANNON.Vec3(pivot2.x, pivot2.z, pivot2.y));
            this._world.addConstraint(constraint);
            
            return true;*/
        };

        GoblinPlugin.prototype.dispose = function () {
            /*while (this._registeredMeshes.length) {
            this.unregisterMesh(this._registeredMeshes[0].mesh);
            }*/
        };

        GoblinPlugin.prototype.isSupported = function () {
            return window.Goblin !== undefined;
        };
        return GoblinPlugin;
    })();
    BABYLON.GoblinPlugin = GoblinPlugin;
})(BABYLON || (BABYLON = {}));
//# sourceMappingURL=babylon.goblinPlugin.js.map
