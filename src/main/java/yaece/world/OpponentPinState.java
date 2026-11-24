package yaece.world;

/** Immutable per-opponent pin guard status exposed in {@link WorldSnapshot}. */
public record OpponentPinState(
    int trackId,
    PinStatus status,
    double activePinSeconds,
    double cooldownElapsedSeconds,
    boolean backoffRequired,
    double separationMeters) {}
