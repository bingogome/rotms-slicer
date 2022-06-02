def utilNumStrFormat(n, decimal=5, zerofill=10):
    """
    Format numbers into string at decimal and fill with zero
    """
    return ("{:."+str(decimal)+"f}").format(n).zfill(zerofill)
