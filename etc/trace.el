;;
;; Code to make working with vxWorks a little easier
;;  Robert Lupton (rhl@astro.princeton.edu)
;;
(defun cccc2name (&optional point mark)
  "Convert any hex strings in the region to text, assuming that they've
be written as e.g. printf(\"0x%x%x\", ((int *)str)[0], ((int *)str)[1]);

Such strings are often written by Ron Rechenmacher's TRACE utility to
get around the danger that vxWorks may have trashed the strings in question
by the time that they are written out.

If called with no arguments, all of the buffer following a line consisting of
traceShow
is processed.
"
  (interactive "r")

  (if (not point)
      (save-excursion
	(goto-char (point-min))
	(if (not (re-search-forward "^traceShow\\s-*$" mark t))
	    (error "I cannot find the \"traceShow\" line"))
	(setq point (match-beginning 0))
	(setq mark (point-max))))

  (let ( (hexstr)			; the string to be replaced
	 (c) )				; a character in the string as an int
    (save-excursion
      (goto-char point)
      (while (re-search-forward "0x\\([0-9a-f][0-9a-f][0-9a-f][0-9a-f]+\\)" mark t)
	(progn
	  (goto-char (match-beginning 1))
	  (setq hexstr (buffer-substring (match-beginning 1) (match-end 1)))
	  (setq c -1)
	  (replace-match "" t t)	; remove original string
	  (while (and (not (= c 0)) (>= (string-bytes hexstr) 2))
	    (progn
	      (setq c (string-to-number (substring hexstr 0 2) 16))
	      (if (not (= c 0))		; not NUL
		  (progn
		    (if (or (< c 32) (> c 126)) ; not printable
			(setq c ?.))
		    (insert-string (format "%c" c))))
		(setq hexstr (substring hexstr 2))) ; trim 2 digits and continue
	      ))
	  (goto-char (match-end 1)))))
  (message "Done"))
