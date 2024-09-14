package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class Promise {
    public abstract boolean isResolved();

    protected abstract void i_then(Lambda callback);

    public Promise then(Lambda callback) {
        i_then(callback);
        return Promise.immediate();
    }

    public Promise then(Getter<Promise> promiseGetter) {
        if (isResolved()) {
            return promiseGetter.get();
        } else {
            SimplePromise returnPromise = new SimplePromise();

            i_then(() -> {
                Promise promise = promiseGetter.get();
                promise.i_then(() -> {
                    returnPromise.resolve();
                });
            });

            return returnPromise;
        }
    }

    public static Promise timeout(double seconds) {
        SimplePromise prom = new SimplePromise();

        CommandScheduler.getInstance().schedule(Commands.waitSeconds(seconds).andThen(new Command() {
            @Override
            public void execute() {
                prom.resolve();
                cancel();
            }
        }));
        return prom;
    }

    public static Promise all(Promise... proms) {
        var resolved = new Container<Integer>(0);
        var all = new SimplePromise();
        for (var prom : proms) {
            prom.i_then(() -> {
                resolved.val++;
                if (resolved.val == proms.length)
                    all.resolve();
            });
        }
        return all;
    }

    // Creates and resolves a promise immediately.
    public static SimplePromise immediate() {
        SimplePromise promise = new SimplePromise();
        promise.resolve();
        return promise;
    }
}
