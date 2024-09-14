package frc.lib.util;

public interface Mapper<From, To> {
    public To map(From p);
}