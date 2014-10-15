module BABYLON {
    declare var Goblin;
    declare var window;

    export class GoblinPlugin implements IPhysicsEnginePlugin {
        private _world: any;
        private _registeredMeshes = [];

        public initialize(iterations: number = 10): void {
            this._world = new Goblin.World(new Goblin.BasicBroadphase(), new Goblin.NearPhase(), new Goblin.IterativeSolver());
            this._world.solver.max_iterations = iterations;
        }

        public runOneStep(delta: number): void {
            this._world.step(delta);

            for (var index = 0; index < this._registeredMeshes.length; index++) {
                var registeredMesh = this._registeredMeshes[index];

                if (registeredMesh.isChild) {
                    continue;
                }

                // Body position
                var bodyX = registeredMesh.body.position.x,
                    bodyY = registeredMesh.body.position.y,
                    bodyZ = registeredMesh.body.position.z;

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
        }

        public setGravity(gravity: Vector3): void {
            this._world.gravity.set(gravity.x, gravity.z, gravity.y);
        }

        public registerMesh(mesh: AbstractMesh, impostor: number, options?: PhysicsBodyCreationOptions): any {
            this.unregisterMesh(mesh);

            mesh.computeWorldMatrix(true);

            var body;
            switch (impostor) {
                case BABYLON.PhysicsEngine.SphereImpostor:
                    var bbox = mesh.getBoundingInfo().boundingBox;
                    var radiusX = bbox.maximumWorld.x - bbox.minimumWorld.x;
                    var radiusY = bbox.maximumWorld.y - bbox.minimumWorld.y;
                    var radiusZ = bbox.maximumWorld.z - bbox.minimumWorld.z;

                    body = new Goblin.RigidBody(
                        new Goblin.SphereShape( Math.max( radiusX, radiusY, radiusZ ) / 2 ),
                        options.mass
                    );
                    break;
                case BABYLON.PhysicsEngine.BoxImpostor:
                    bbox = mesh.getBoundingInfo().boundingBox;
                    var min = bbox.minimumWorld;
                    var max = bbox.maximumWorld;
                    var box = max.subtract(min).scale(0.5);

                    body = new Goblin.RigidBody(
                        new Goblin.BoxShape( box.x, box.y, box.z ),
                        options.mass
                    );
                    break;
                case BABYLON.PhysicsEngine.PlaneImpostor:
                    bbox = mesh.getBoundingInfo().boundingBox;
                    var min = bbox.minimumWorld;
                    var max = bbox.maximumWorld;
                    var box = max.subtract(min).scale(0.5);

                    body = new Goblin.RigidBody(
                        new Goblin.PlaneShape( 1, box.y / 2, box.z / 2 ),
                        options.mass
                    );
                    break;
                case BABYLON.PhysicsEngine.MeshImpostor:
                    var rawVerts = mesh.getVerticesData(BABYLON.VertexBuffer.PositionKind);
                    debugger;

                    body = new Goblin.RigidBody(
                        new Goblin.ConvexShape(
                            rawVerts.map(function(vertex){
                                var transformed = BABYLON.Vector3.Zero();
                                BABYLON.Vector3.TransformNormalFromFloatsToRef(rawVerts[i], rawVerts[i + 1], rawVerts[i + 2], mesh.getWorldMatrix(), transformed);

                                return new Goblin.Vector3( transformed.x, transformed.y, transformed.z )
                            })
                        ),
                        options.mass
                    );
                    break;
            }

            body.friction = options.friction;
            body.restitution = options.restitution;

            var initialRotation: Quaternion = null;

            if (mesh.rotationQuaternion) {
                initialRotation = mesh.rotationQuaternion.clone();
                mesh.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
            } else {
                initialRotation = BABYLON.Quaternion.RotationYawPitchRoll( mesh.rotation.y, mesh.rotation.x, mesh.rotation.z );
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
        }

        public registerMeshesAsCompound(parts: PhysicsCompoundBodyPart[], options: PhysicsBodyCreationOptions): any {
            var compoundShape = new Goblin.CompoundShape();

            for (var index = 0; index < parts.length; index++) {
                var mesh = parts[index].mesh;

                var shape = this.registerMesh(mesh, parts[index].impostor);

                if (index == 0) { // Parent
                    compoundShape.addChildShape(shape, new Goblin.Vector3(0, 0, 0), new Goblin.Quaternion(0, 0, 0, 1));
                } else {
                    compoundShape.addChildShape(
                        shape,
                        new Goblin.Vector3(mesh.position.x, mesh.position.z, mesh.position.y),
                        new Goblin.Quaternion(mesh.rotationQuaternion.x, mesh.rotationQuaternion.y, mesh.rotationQuaternion.z, mesh.rotationQuaternion.w)
                    );
                }
            }

            var body = new Goblin.RigidBody( shape, options.mass );
            body.friction = options.friction;
            body.restitution = options.restitution;

            var initialRotation: Quaternion = null;

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
        }

        public unregisterMesh(mesh: AbstractMesh): void {
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
        }

        public applyImpulse(mesh: AbstractMesh, force: Vector3, contactPoint: Vector3): void {
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
        }

        public createLink(mesh1: AbstractMesh, mesh2: AbstractMesh, pivot1: Vector3, pivot2: Vector3): boolean {
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
        }

        public dispose(): void {
            /*while (this._registeredMeshes.length) {
                this.unregisterMesh(this._registeredMeshes[0].mesh);
            }*/
        }

        public isSupported(): boolean {
            return window.Goblin !== undefined;
        }
    }
}