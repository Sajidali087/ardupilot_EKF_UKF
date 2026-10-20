#!/usr/bin/env python3

# flake8: noqa

'''
check UKF shadow estimator output in Replay logs (copy of check_replay.py with the UKF messages)

  check_replay_ukf.py LOG
      original UKF messages (core c) against replayed ones (core c+100); for logs
      recorded with UKF_ENABLE=1

  check_replay_ukf.py --compare LOG_A LOG_B
      determinism: the replayed UKF messages (core >= 100) of two Replay outputs of the
      same input log must be identical, message for message

  --types UKF1,UKF2,UKF3,UKFN restricts either mode to these messages (e.g. to compare
  with a log written before UKFD existed)
'''

UKF_LIST = ['UKF1', 'UKF2', 'UKF3', 'UKFN', 'UKFD', 'UKFH', 'UKFM', 'UKFP', 'UKFS']


def msg_key(m):
    '''UKFN carries several updates per frame, one per measurement type T; UKFM one record per module M'''
    if m.get_type() == 'UKFN':
        return (m.C, m.T)
    if m.get_type() == 'UKFM':
        return (m.C, m.M)
    return m.C


def fields_match(mtype, m, mb, progress, ignores, accuracy, skip=('C',)):
    errors = 0
    for f in m._fieldnames:
        if f in skip:
            continue
        if "%s.%s" % (mtype, f) in ignores:
            continue
        v1 = getattr(m, f)
        v2 = getattr(mb, f)
        ok = v1 == v2
        if not ok and accuracy > 0:
            avg = (v1+v2)*0.5
            margin = accuracy*0.01*avg
            if abs(v1-v2) <= abs(margin):
                ok = True
        if not ok:
            errors += 1
            progress("Mismatch in field %s.%s: %s %s" % (mtype, f, str(v1), str(v2)))
    return errors


def check_log(logfile, progress=print, verbose=False, accuracy=0.0, ignores=set()):
    '''original (core < 100) against replayed (core >= 100) UKF output in one log'''
    from pymavlink import mavutil
    progress("Processing log %s" % logfile)
    failure = 0
    errors = 0
    count = 0
    base_count = 0
    counts = {}
    base_counts = {}
    base = {m: {} for m in UKF_LIST}

    mlog = mavutil.mavlink_connection(logfile)
    while True:
        m = mlog.recv_match(type=UKF_LIST)
        if m is None:
            break
        mtype = m.get_type()
        counts.setdefault(mtype, 0)
        base_counts.setdefault(mtype, 0)
        if m.C < 100:
            base[mtype][msg_key(m)] = m
            base_count += 1
            base_counts[mtype] += 1
            continue
        key = msg_key(m)
        key = (key[0]-100, key[1]) if isinstance(key, tuple) else key-100
        mb = base[mtype].get(key)
        if mb is None:
            continue
        count += 1
        counts[mtype] += 1
        e = fields_match(mtype, m, mb, progress, ignores, accuracy)
        if e:
            errors += e
            progress(mb)
            progress(m)
    progress("Processed %u/%u messages, %u errors" % (count, base_count, errors))
    if verbose:
        for mtype in counts.keys():
            progress("%s %u/%u %d" % (mtype, counts[mtype], base_counts[mtype], base_counts[mtype]-counts[mtype]))
    count_delta = abs(count - base_count)
    if count == 0 or count_delta > 100:
        progress("count=%u count_delta=%u" % (count, count_delta))
        failure += 1
    return failure == 0 and errors == 0


ONLY_CORE = None


def replayed_ukf(logfile):
    '''generator of replayed UKF messages (core >= 100) in log order; with --core only that core'''
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(logfile)
    while True:
        m = mlog.recv_match(type=UKF_LIST)
        if m is None:
            return
        if m.C >= 100 and (ONLY_CORE is None or m.C == 100 + ONLY_CORE):
            yield m


def compare_logs(log_a, log_b, progress=print, verbose=False, accuracy=0.0, ignores=set()):
    '''determinism: replayed UKF messages of two Replay outputs must be identical'''
    progress("Comparing %s with %s" % (log_a, log_b))
    errors = 0
    count = 0
    counts = {}
    ga = replayed_ukf(log_a)
    gb = replayed_ukf(log_b)
    while True:
        ma = next(ga, None)
        mb = next(gb, None)
        if ma is None and mb is None:
            break
        if ma is None or mb is None:
            progress("Different number of UKF messages (one log ends after %u)" % count)
            errors += 1
            break
        mtype = ma.get_type()
        if mtype != mb.get_type():
            progress("Message order differs at %u: %s %s" % (count, mtype, mb.get_type()))
            errors += 1
            break
        count += 1
        counts[mtype] = counts.get(mtype, 0) + 1
        e = fields_match(mtype, ma, mb, progress, ignores, accuracy, skip=())
        if e:
            errors += e
            progress(ma)
            progress(mb)
    progress("Compared %u messages, %u errors" % (count, errors))
    if verbose:
        for mtype in counts.keys():
            progress("%s %u" % (mtype, counts[mtype]))
    return count > 0 and errors == 0


if __name__ == '__main__':
    import sys
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--compare", action='store_true', help="compare the replayed UKF output of two logs")
    parser.add_argument("--verbose", action='store_true', help="verbose output")
    parser.add_argument("--accuracy", type=float, default=0.0, help="accuracy percentage for match")
    parser.add_argument("--ignore-field", action='append', default=[], help="ignore message field when comparing")
    parser.add_argument("--types", default=None, help="comma-separated UKF messages to check (default: all)")
    parser.add_argument("--core", type=int, default=None, help="--compare only: compare only this UKF core (0..2)")
    parser.add_argument("logs", metavar="LOG", nargs="+")

    args = parser.parse_args()
    if args.types:
        UKF_LIST[:] = args.types.split(',')
    ONLY_CORE = args.core

    failed = False
    if args.compare:
        if len(args.logs) != 2:
            print("--compare needs exactly two logs")
            sys.exit(2)
        if not compare_logs(args.logs[0], args.logs[1], print, args.verbose, accuracy=args.accuracy, ignores=args.ignore_field):
            failed = True
    else:
        for filename in args.logs:
            if not check_log(filename, print, args.verbose, accuracy=args.accuracy, ignores=args.ignore_field):
                failed = True

    if failed:
        print("FAILED")
        sys.exit(1)
    print("Passed")
    sys.exit(0)
